/* sv4e_mode_loader.cpp — JSON-driven SV4E mode loader.
 * TODO: the schema accepts an `fps` field but downstream AE
 * timing is still 30 fps only (SV4E_VMAX_30FPS=4500, SV4E_HMAX=0x0226
 * in mipi_generator_sv4e.c, SV4E.c:71-72). Non-30 fps modes get
 * correct CSI traffic and wrong AE math; logs WARN per such mode.
 */
#include "sv4e_mode_loader.h"
#include <isi/isi_common.h>  /* ISI_FPS_ACCURACY */
#include <ebase/trace.h>
#include <json/json.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

/* Local tracers — route through TRACE so ISP_LOG_LEVEL gates them and
 * lines reach the same sinks as the rest of SV4E (not stderr, which
 * gets swallowed when isp_media_server is daemonized).
 */
CREATE_TRACER(SV4E_LOADER_WARN,  "SV4E_LOADER: ", WARNING, 1);
CREATE_TRACER(SV4E_LOADER_ERROR, "SV4E_LOADER: ", ERROR,   1);

namespace {

const char *resolve_path()
{
    const char *env = ::getenv("HAILO_SV4E_MODES_JSON");
    if (env && *env)
        return env;
    return "/etc/hailo/sv4e_modes.json";
}

int bayer_from_string(const std::string &s, uint32_t *out)
{
    if (s == "RGGB") { *out = BAYER_RGGB; return 0; }
    if (s == "GBRG") { *out = BAYER_GBRG; return 0; }
    if (s == "GRBG") { *out = BAYER_GRBG; return 0; }
    if (s == "BGGR") { *out = BAYER_BGGR; return 0; }
    return -EINVAL;
}

int hdr_from_string(const std::string &s, uint32_t *out)
{
    if (s == "linear") { *out = SENSOR_MODE_LINEAR; return 0; }
    return -EINVAL; /* HDR variants deferred */
}

bool within_bounds(uint32_t w, uint32_t h, uint32_t bit_width)
{
    return w >= 1 && w <= 3840 && h >= 1 && h <= 2160 && bit_width == 12;
}

int parse_one(const Json::Value &j, vvsensor_mode_s *m)
{
    static const char *const required[] = {
        "width", "height", "fps", "bayer",
        "bit_width", "hdr_mode", "ae_info", "index"
    };
    for (const char *field : required) {
        if (!j.isMember(field)) {
            TRACE(SV4E_LOADER_WARN, "missing required field '%s'\n", field);
            return -EINVAL;
        }
    }

    std::memset(m, 0, sizeof(*m));
    m->index            = j["index"].asUInt();
    m->size.bounds_width  = j["width"].asUInt();
    m->size.bounds_height = j["height"].asUInt();
    m->size.width       = m->size.bounds_width;
    m->size.height      = m->size.bounds_height;
    m->size.top         = 0;
    m->size.left        = 0;
    m->fps              = j["fps"].asUInt() * ISI_FPS_ACCURACY;
    m->bit_width        = j["bit_width"].asUInt();

    if (!within_bounds(m->size.width, m->size.height, m->bit_width)) {
        TRACE(SV4E_LOADER_WARN,
              "out-of-bounds: width=%u height=%u bit_width=%u\n",
              m->size.width, m->size.height, m->bit_width);
        return -EINVAL;
    }

    if (bayer_from_string(j["bayer"].asString(), &m->bayer_pattern)) {
        TRACE(SV4E_LOADER_WARN, "unknown bayer pattern '%s'\n",
              j["bayer"].asString().c_str());
        return -EINVAL;
    }
    if (hdr_from_string(j["hdr_mode"].asString(), &m->hdr_mode)) {
        TRACE(SV4E_LOADER_WARN, "unsupported hdr_mode '%s'\n",
              j["hdr_mode"].asString().c_str());
        return -EINVAL;
    }

    const Json::Value &ae = j["ae_info"];
    m->ae_info.one_line_exp_time_ns =
        ae.get("one_line_exp_time_ns", 7407).asUInt();
    m->ae_info.max_integration_time =
        ae.get("max_integration_time", 4497).asUInt();
    m->ae_info.min_integration_time =
        ae.get("min_integration_time", 1).asUInt();
    m->ae_info.max_gain = ae.get("max_gain", 3981).asUInt();
    m->ae_info.min_gain = ae.get("min_gain", 1).asUInt();
    m->ae_info.integration_accuracy = ae.get("integration_accuracy", 1).asUInt();
    m->ae_info.gain_accuracy = ae.get("gain_accuracy", 1).asUInt();
    m->ae_info.cur_fps  = ae.get("cur_fps", 30).asUInt();

    if (m->fps != 30 * ISI_FPS_ACCURACY) {
        TRACE(SV4E_LOADER_WARN,
              "mode %u fps=%u != 30; AE timing will be wrong "
              "(non-30 fps support pending)\n",
              m->index, j["fps"].asUInt());
    }
    return 0;
}

} /* anon namespace */

/* sv4e_load_modes_from_json: parse JSON mode table from
 * $HAILO_SV4E_MODES_JSON (or /etc/hailo/sv4e_modes.json).
 *
 * Ownership: on success, out_arr points to a calloc'd array of
 * out_n vvsensor_mode_s entries. The CALLER owns the buffer and
 * must free(*out_arr) when done. On any failure, the output params
 * are untouched and nothing is leaked (fail-closed).
 */
extern "C" int sv4e_load_modes_from_json(struct vvsensor_mode_s **out_arr,
                                          size_t *out_n)
{
    if (!out_arr || !out_n)
        return -EINVAL;

    const char *path = resolve_path();
    std::ifstream f(path);
    if (!f.good())
        return -ENOENT;

    Json::Value root;
    Json::CharReaderBuilder b;
    std::string errs;
    if (!Json::parseFromStream(b, f, &root, &errs)) {
        TRACE(SV4E_LOADER_ERROR, "parse failed: %s\n", errs.c_str());
        return -EINVAL;
    }
    if (!root.isMember("modes") || !root["modes"].isArray() ||
        root["modes"].empty())
        return -EINVAL;

    size_t n = root["modes"].size();
    std::vector<vvsensor_mode_s> tmp(n);
    for (size_t i = 0; i < n; ++i) {
        if (parse_one(root["modes"][(Json::ArrayIndex)i], &tmp[i]) != 0) {
            TRACE(SV4E_LOADER_ERROR,
                  "mode %zu invalid; aborting load\n", i);
            return -EINVAL;
        }
    }
    /* Reject duplicate indices — Sv4e_IsiHalEnumModeIss is first-match,
     * so duplicates silently shadow.
     */
    for (size_t i = 0; i < n; ++i) {
        for (size_t k = i + 1; k < n; ++k) {
            if (tmp[i].index == tmp[k].index) {
                TRACE(SV4E_LOADER_ERROR,
                      "duplicate index %u at entries %zu and %zu\n",
                      tmp[i].index, i, k);
                return -EINVAL;
            }
        }
    }
    /* caller-owned: free(*out_arr) when done */
    auto *arr = static_cast<vvsensor_mode_s *>(
        std::calloc(n, sizeof(vvsensor_mode_s)));
    if (!arr)
        return -ENOMEM;
    std::memcpy(arr, tmp.data(), n * sizeof(vvsensor_mode_s));
    *out_arr = arr;
    *out_n   = n;
    return 0;
}
