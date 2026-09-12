#!/usr/bin/env bash
# The host test suite: every pure-logic file compiled with gcc and the
# sanitizers, the way jradio's tests are run, so the protocol is proven on a
# PC before either board sees a byte of it.
set -euo pipefail

project_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
build_dir=$(mktemp -d /tmp/jradio-bt-host-tests.XXXXXX)
trap 'rm -rf -- "${build_dir}"' EXIT

cc=${CC:-gcc}
flags=(-std=c17 -Wall -Wextra -Werror -Wpedantic -fsanitize=address,undefined -fno-omit-frame-pointer)
includes=(-I"${project_dir}/components/jbt_proto/include")

run_test() {
    local name=$1
    shift
    "${cc}" "${flags[@]}" "${includes[@]}" "$@" -o "${build_dir}/${name}"
    ASAN_OPTIONS=detect_leaks=1 "${build_dir}/${name}"
}

run_test jbt_proto "${project_dir}/tests/test_jbt_proto.c" "${project_dir}/components/jbt_proto/jbt_proto.c"

# The PC tool is the protocol's second implementation; its selftest pins the
# same wire vector the C test does.
python3 "${project_dir}/tools/jbt.py" selftest

echo "All host tests passed."
