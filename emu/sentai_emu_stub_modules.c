// B8.3: empty `sentai` module so the linker can resolve mp_module_sentai
// (referenced by the shared genhdr/moduledefs.h) without pulling production
// bindings into the emulator REPL target.

#include "py/obj.h"
#include "py/runtime.h"

static const mp_rom_map_elem_t sentai_emu_module_globals_table[] = {
    {MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sentai)},
};
static MP_DEFINE_CONST_DICT(sentai_emu_module_globals,
                            sentai_emu_module_globals_table);

const mp_obj_module_t mp_module_sentai = {
    .base = {&mp_type_module},
    .globals = (mp_obj_dict_t *)&sentai_emu_module_globals,
};
