#!/usr/bin/env bash
# Build ArduPilot's bundled Lua on the host, syntax-check F35B.lua with it,
# then run the offline behavioral scenarios in harness.lua.
#
# usage: libraries/AP_Scripting/applets/F35B_offline/run.sh [path/to/F35B.lua]
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
APPLETS="$(dirname "$HERE")"
LUA_SRC="$(cd "$APPLETS/../lua/src" && pwd)"
SCRIPT="${1:-$APPLETS/F35B.lua}"
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

cp -r "$LUA_SRC" "$BUILD/src"
cd "$BUILD/src"
# strip the four ArduPilot-specific hooks so the tree builds standalone
sed -i '/AP_Filesystem\/posix_compat.h/d' lprefix.h
sed -i '/AP_Scripting\/lua_common_defs.h/d' luaconf.h
sed -i 's#<AP_HAL/ap_setjmp.h>#<setjmp.h>#' ldo.c
sed -i '/^static void findloader/,/luaL_Buffer msg;/ s/^  luaL_Buffer msg;/  int i;\n  luaL_Buffer msg;/' loadlib.c

cat > "$BUILD/aprun.c" <<'EOF'
/* -c FILE...  : syntax-check each file
   SCRIPT ARGS : run SCRIPT with the standard libraries, ARGS in `arg` */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lua.h"
#include "lauxlib.h"
#include "lualib.h"
static void *l_alloc(void *ud, void *ptr, size_t osize, size_t nsize) {
  (void)ud; (void)osize;
  if (nsize == 0) { free(ptr); return NULL; }
  return realloc(ptr, nsize);
}
lua_State *luaL_newstate(void) { return lua_newstate(l_alloc, NULL); }
void lua_abort(void) { fprintf(stderr, "lua_abort\n"); abort(); }
int lua_get_current_env_ref(void) { return LUA_NOREF; }
int main(int argc, char **argv) {
  if (argc > 1 && strcmp(argv[1], "-c") == 0) {
    int bad = 0;
    for (int i = 2; i < argc; i++) {
      lua_State *L = luaL_newstate();
      if (luaL_loadfile(L, argv[i]) != LUA_OK) { printf("syntax FAIL %s\n", lua_tostring(L, -1)); bad = 1; }
      else printf("syntax OK   %s\n", argv[i]);
      lua_close(L);
    }
    return bad;
  }
  lua_State *L = luaL_newstate();
  luaL_openlibs(L);
  lua_newtable(L);
  for (int i = 2; i < argc; i++) { lua_pushstring(L, argv[i]); lua_rawseti(L, -2, i - 1); }
  lua_setglobal(L, "arg");
  if (luaL_dofile(L, argv[1]) != LUA_OK) { fprintf(stderr, "ERR %s\n", lua_tostring(L, -1)); return 1; }
  return 0;
}
EOF

gcc -O1 -w -DLUA_USE_POSIX -DSCRIPTING_DIRECTORY='"./scripts"' -I"$BUILD/src" \
    -o "$BUILD/aprun" "$BUILD/aprun.c" $(ls "$BUILD"/src/l*.c | grep -vE '/(luac|lua)\.c$') -lm

"$BUILD/aprun" -c "$SCRIPT"
"$BUILD/aprun" "$HERE/harness.lua" "$SCRIPT"
