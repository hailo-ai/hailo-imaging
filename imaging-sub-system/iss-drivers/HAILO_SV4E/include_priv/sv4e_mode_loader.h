/* sv4e_mode_loader.h — JSON-driven SV4E mode loader. Returns 0 on success
 * with *out_arr and *out_n pointing to a calloc'd, caller-owned array of
 * modes loaded from $HAILO_SV4E_MODES_JSON or /etc/hailo/sv4e_modes.json.
 * Returns -ENOENT if no file present; -EINVAL on parse/validation failure.
 */
#ifndef SV4E_MODE_LOADER_H
#define SV4E_MODE_LOADER_H

#include "vvsensor.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

int sv4e_load_modes_from_json(struct vvsensor_mode_s **out_arr,
                              size_t *out_n);

#ifdef __cplusplus
}
#endif

#endif /* SV4E_MODE_LOADER_H */
