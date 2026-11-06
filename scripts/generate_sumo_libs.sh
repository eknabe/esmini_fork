#!/bin/bash

# Build headless SUMO static libraries for embedded integration in esmini.
# Run from a scratch directory with Bash (Git Bash on Windows). No system
# installation or administrator privileges are required.
#
# Prerequisites: Git, CMake >= 3.18 and < 4, Python 3, a C++17 compiler,
# esmini's static PROJ package (including SQLite), and external 7-Zip.
# USE_SUMO requires USE_PROJ in esmini.
# Windows defaults to Visual Studio 2022 with the v142 x64 toolset installed.
#
# Usage: bash /path/to/esmini/scripts/generate_sumo_libs.sh
# Environment overrides:
#   WORK_DIR             Source/build root (default: current directory)
#   PARALLEL_BUILDS      Parallel build jobs (default: 8)
#   GENERATOR            CMake generator (Windows: Visual Studio 17 2022)
#   GENERATOR_TOOLSET    Windows toolset (default: v142)
#   MACOS_ARCHITECTURES  macOS architectures (default: arm64;x86_64)
#   SUMO_SOURCE_DIR      Optional clean Git checkout at exactly v1_28_0
#   PROJ_ROOT            esmini PROJ package root containing include/ and lib/
#                        (default: this repository's externals/proj/<platform>)
#   SEVEN_ZIP            7-Zip executable (default: PATH or Windows install)
#
# Debug and Release are built separately on Windows/Linux; macOS builds
# universal Release archives. Rerunning resumes the builds through CMake,
# stages a fresh platform package, and replaces the versioned .7z archive.
# PROJ/SQLite are reused from esmini, not included in the SUMO package.

set -Eeuo pipefail
trap 'printf "ERROR: SUMO build failed at line %s. See the output above.\n" "$LINENO" >&2' ERR

ZLIB_VERSION=v1.2.12
XERCES_VERSION=v3.2.5
SUMO_VERSION=v1_28_0
PARALLEL_BUILDS=${PARALLEL_BUILDS:-8}

fail() {
    printf 'ERROR: %s\n' "$*" >&2
    exit 1
}

[[ $# == 0 ]] || fail "No command-line arguments are supported; use the documented environment overrides."
[[ "$PARALLEL_BUILDS" =~ ^[1-9][0-9]*$ ]] || fail "PARALLEL_BUILDS must be a positive integer."

generator_args=()
platform_args=()
configurations=(Debug Release)
case "$OSTYPE" in
    msys*|cygwin*)
        platform=v10
        lib_prefix=
        lib_ext=lib
        GENERATOR=${GENERATOR:-Visual Studio 17 2022}
        GENERATOR_TOOLSET=${GENERATOR_TOOLSET:-v142}
        [[ "$GENERATOR" == "Visual Studio "* ]] || fail "Windows requires a Visual Studio generator."
        generator_args=(-A x64 -T "$GENERATOR_TOOLSET")
        platform_args=(
            -DCMAKE_POLICY_DEFAULT_CMP0091=NEW
            '-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded$<$<CONFIG:Debug>:Debug>DLL'
        )
        # Preserve CMake's default exception/ABI flags while parallelizing
        # compilation within each Visual Studio library target.
        export CL="${CL:+$CL }-MP${PARALLEL_BUILDS}"
        build_platform="${platform}-${GENERATOR_TOOLSET}"
        ;;
    linux*)
        platform=linux
        lib_prefix=lib
        lib_ext=a
        GENERATOR=${GENERATOR:-Unix Makefiles}
        build_platform=$platform
        ;;
    darwin*)
        platform=mac
        lib_prefix=lib
        lib_ext=a
        GENERATOR=${GENERATOR:-Unix Makefiles}
        configurations=(Release)
        platform_args=("-DCMAKE_OSX_ARCHITECTURES=${MACOS_ARCHITECTURES:-arm64;x86_64}")
        build_platform=$platform
        ;;
    *) fail "Unsupported OSTYPE: $OSTYPE" ;;
esac

for tool in git cmake; do
    command -v "$tool" >/dev/null || fail "Required tool not found: $tool"
done

cmake_version=$(cmake --version | sed -n '1s/^cmake version //p')
[[ "$cmake_version" =~ ^([0-9]+)\.([0-9]+) ]] ||
    fail "Unable to determine the CMake version."
cmake_major=${BASH_REMATCH[1]}
cmake_minor=${BASH_REMATCH[2]}
(( cmake_major == 3 && cmake_minor >= 18 )) ||
    fail "CMake 3.18 through 3.x is required by the pinned dependency projects; found $cmake_version."

python_command=
for candidate in python python3; do
    if command -v "$candidate" >/dev/null &&
        "$candidate" -c 'import sys; sys.exit(0 if sys.version_info.major == 3 else 1)'; then
        python_command=$(command -v "$candidate")
        break
    fi
done
[[ -n "$python_command" ]] || fail "Python 3 must be available as python or python3."

if [[ -z "${SEVEN_ZIP:-}" ]]; then
    for candidate in 7z 7zz 7za "/c/Program Files/7-Zip/7z.exe"; do
        if command -v "$candidate" >/dev/null; then
            SEVEN_ZIP=$candidate
            break
        fi
    done
fi
[[ -n "${SEVEN_ZIP:-}" ]] && command -v "$SEVEN_ZIP" >/dev/null ||
    fail "7-Zip not found. Set SEVEN_ZIP to the 7z executable."

# Native CMake needs Windows paths even inside -D arguments and lists.
cmake_path() {
    if [[ "$platform" == v10 ]]; then
        cygpath -am "$1"
    else
        printf '%s\n' "$1"
    fi
}

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
PROJ_ROOT=${PROJ_ROOT:-$script_dir/../externals/proj/$platform}
[[ -s "$PROJ_ROOT/include/proj.h" ]] ||
    fail "Static PROJ package not found at $PROJ_ROOT. Set PROJ_ROOT to esmini's platform PROJ package."
PROJ_ROOT=$(cd "$PROJ_ROOT" && pwd -P)
for configuration in "${configurations[@]}"; do
    proj_suffix=
    [[ "$configuration" != Debug ]] || proj_suffix=d
    for library in proj sqlite3; do
        [[ -s "$PROJ_ROOT/lib/${lib_prefix}${library}${proj_suffix}.$lib_ext" ]] ||
            fail "Missing $configuration static library in PROJ_ROOT: ${lib_prefix}${library}${proj_suffix}.$lib_ext"
    done
done

check_source() {
    local directory=$1 version=$2 expected
    [[ -e "$directory/.git" ]] || fail "Not a Git checkout: $directory"
    expected=$(git -C "$directory" rev-parse --verify "refs/tags/${version}^{commit}") ||
        fail "Missing release tag $version in $directory."
    [[ "$(git -C "$directory" rev-parse HEAD)" == "$expected" ]] ||
        fail "$directory must be at $version. Use a separate release checkout; this script will not switch it."
    [[ -z "$(git -C "$directory" status --porcelain)" ]] ||
        fail "Source checkout is not clean: $directory"
}

if [[ -n "${SUMO_SOURCE_DIR:-}" ]]; then
    check_source "$SUMO_SOURCE_DIR" "$SUMO_VERSION"
    SUMO_SOURCE_DIR=$(cd "$SUMO_SOURCE_DIR" && pwd -P)
fi

mkdir -p "${WORK_DIR:-$PWD}"
WORK_DIR=$(cd "${WORK_DIR:-$PWD}" && pwd -P)
source_root="$WORK_DIR/sources"
build_root="$WORK_DIR/build/$build_platform"
mkdir -p "$source_root"

fetch_source() {
    local name=$1 version=$2 url=$3 directory="$source_root/$1-$2"
    if [[ ! -e "$directory" ]]; then
        git clone --depth 1 --branch "$version" --no-checkout "$url" "$directory"
        case "$name" in
            zlib)
                # zlib's CMake renames tracked zconf.h in out-of-source builds.
                # Omit it so only the generated build-tree header is used.
                git -C "$directory" sparse-checkout set --no-cone '/*' '!/zconf.h'
                ;;
            sumo)
                git -C "$directory" sparse-checkout set --no-cone \
                    '/*' '!/*/' '/src/' '/build_config/' '/tools/' '/data/'
                ;;
        esac
        git -C "$directory" -c advice.detachedHead=false checkout --detach "$version"
    fi
    check_source "$directory" "$version"
}

fetch_source zlib "$ZLIB_VERSION" https://github.com/madler/zlib.git
fetch_source xerces-c "$XERCES_VERSION" https://github.com/apache/xerces-c.git
if [[ -z "${SUMO_SOURCE_DIR:-}" ]]; then
    fetch_source sumo "$SUMO_VERSION" https://github.com/eclipse-sumo/sumo.git
    SUMO_SOURCE_DIR="$source_root/sumo-$SUMO_VERSION"
fi
zlib_source="$source_root/zlib-$ZLIB_VERSION"
xerces_source="$source_root/xerces-c-$XERCES_VERSION"

# SUMO 1.28's sumolibs/commonvehiclelibs/commonlibs plus their static dependencies.
# libsumostatic alone does not build the simulation libraries.
sumo_modules=(
    traciserver netload microsim_cfmodels microsim_engine microsim_lcmodels
    microsim_devices microsim_trigger microsim_output microsim_transportables
    microsim_actions microsim_traffic_lights microsim mesosim utils_emissions
    foreign_phemlight foreign_phemlight_V5 utils_vehicle utils_distribution
    utils_handlers utils_shapes utils_options utils_xml utils_geom utils_common
    utils_importio utils_iodevices utils_traction_wire foreign_tcpip foreign_routingkit
)

sumo_options=(
    -DCHECK_OPTIONAL_LIBS=OFF -DISOLATED_BUILD=ON -DINSTALL_DLL_RUNTIMES=OFF
    -DSUMO_UTILS=OFF -DENABLE_FMI=OFF -DENABLE_NETEDIT=OFF
    -DENABLE_PYTHON_BINDINGS=OFF -DENABLE_JAVA_BINDINGS=OFF
    -DENABLE_CS_BINDINGS=OFF -DENABLE_C_BINDINGS=OFF
    -DENABLE_PROJ=ON -DENABLE_FOX=OFF -DENABLE_EIGEN=OFF -DENABLE_FMT=OFF
    -DENABLE_GTEST=OFF -DENABLE_GDAL=OFF -DENABLE_FFMPEG=OFF -DENABLE_OSG=OFF
    -DENABLE_GL2PS=OFF -DENABLE_JUPEDSIM=OFF -DENABLE_BOOST=OFF
    -DENABLE_PARQUET=OFF -DENABLE_TCMALLOC=OFF -DCCACHE_SUPPORT=OFF
    -DPROFILING=OFF -DPPROF=OFF -DCOVERAGE=OFF
    -DCMAKE_DISABLE_FIND_PACKAGE_Intl=ON -DCMAKE_DISABLE_FIND_PACKAGE_Gettext=ON
    -DCMAKE_DISABLE_FIND_PACKAGE_X11=ON -DCMAKE_DISABLE_FIND_PACKAGE_Freetype=ON
    -DCMAKE_DISABLE_FIND_PACKAGE_SWIG=ON -DCMAKE_DISABLE_FIND_PACKAGE_Java=ON
    -DCOMPILE_DEFINITIONS=XERCES_STATIC_LIBRARY
)
if [[ "$platform" == v10 ]]; then
    sumo_options+=('-DCOMPILE_DEFINITIONS=XERCES_STATIC_LIBRARY;PROJ_DLL=')
fi

configure() {
    local source=$1 build=$2
    shift 2
    MSYS2_ARG_CONV_EXCL='*' cmake -S "$(cmake_path "$source")" -B "$(cmake_path "$build")" \
        -G "$GENERATOR" "${generator_args[@]}" "${platform_args[@]}" \
        "-DCMAKE_BUILD_TYPE=$configuration" \
        "-DCMAKE_CONFIGURATION_TYPES=$configuration" \
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        "-DCMAKE_INSTALL_PREFIX=$(cmake_path "$prefix")" \
        -DCMAKE_INSTALL_LIBDIR=lib \
        "-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY=$(cmake_path "$build/lib")" \
        "-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG=$(cmake_path "$build/lib")" \
        "-DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE=$(cmake_path "$build/lib")" \
        "$@"
}

build_targets() {
    local build=$1
    shift
    cmake --build "$(cmake_path "$build")" --config "$configuration" \
        --parallel "$PARALLEL_BUILDS" --target "$@"
}

require_file() {
    [[ -s "$1" ]] || fail "Required build output is missing or empty: $1"
}

for configuration in "${configurations[@]}"; do
    printf '\n---------------- Building %s (%s) ----------------\n' "$configuration" "$build_platform"
    zlib_build="$build_root/zlib-$ZLIB_VERSION/$configuration"
    xerces_build="$build_root/xerces-c-$XERCES_VERSION/$configuration"
    sumo_build="$build_root/sumo-$SUMO_VERSION/$configuration"
    prefix="$sumo_build/dependencies"
    mkdir -p "$prefix/include" "$prefix/lib"

    zlib_name="${lib_prefix}z.$lib_ext"
    xerces_name="${lib_prefix}xerces-c.$lib_ext"
    sumo_name="libsumostatic.$lib_ext"
    proj_suffix=
    if [[ "$platform" == v10 ]]; then
        zlib_name=zlibstatic.lib
        xerces_name=xerces-c_3.lib
    fi
    if [[ "$configuration" == Debug ]]; then
        proj_suffix=d
        sumo_name="libsumostaticD.$lib_ext"
        if [[ "$platform" == v10 ]]; then
            zlib_name=zlibstaticd.lib
            xerces_name=xerces-c_3D.lib
        fi
    fi

    configure "$zlib_source" "$zlib_build"
    build_targets "$zlib_build" zlibstatic
    require_file "$zlib_build/lib/$zlib_name"
    cp -p "$zlib_source/zlib.h" "$zlib_build/zconf.h" "$prefix/include/"
    cp -p "$zlib_build/lib/$zlib_name" "$prefix/lib/"

    configure "$xerces_source" "$xerces_build" \
        -DBUILD_SHARED_LIBS=OFF -Dnetwork=OFF -Dmessage-loader=inmemory \
        -DCMAKE_DISABLE_FIND_PACKAGE_ICU=ON
    build_targets "$xerces_build" xerces-c
    require_file "$xerces_build/lib/$xerces_name"
    # Install only headers/export metadata; the runtime component includes
    # sample executables which are deliberately not built.
    cmake --install "$(cmake_path "$xerces_build")" --config "$configuration" --component development
    cp -p "$xerces_build/lib/$xerces_name" "$prefix/lib/"

    configure "$SUMO_SOURCE_DIR" "$sumo_build" "${sumo_options[@]}" \
        "-DPROJ_INCLUDE_DIR=$(cmake_path "$PROJ_ROOT/include")" \
        "-DPROJ_LIBRARY=$(cmake_path "$PROJ_ROOT/lib/${lib_prefix}proj${proj_suffix}.$lib_ext")" \
        "-DSUMO_LIBRARIES=$(cmake_path "$prefix")" \
        "-DPython_EXECUTABLE=$(cmake_path "$python_command")" \
        "-DZLIB_INCLUDE_DIR=$(cmake_path "$prefix/include")" \
        "-DZLIB_LIBRARY=$(cmake_path "$prefix/lib/$zlib_name")" \
        "-DXercesC_INCLUDE_DIR=$(cmake_path "$prefix/include")" \
        "-DXercesC_LIBRARY=$(cmake_path "$prefix/lib/$xerces_name")"
    build_targets "$sumo_build" libsumostatic "${sumo_modules[@]}"
    require_file "$sumo_build/lib/$sumo_name"
    for module in "${sumo_modules[@]}"; do
        require_file "$sumo_build/lib/${lib_prefix}${module}.$lib_ext"
    done
    printf 'Verified %s static libraries: %s\n' "$configuration" "$sumo_build/lib"
done

printf '\n---------------- Staging esmini package ----------------\n'
stage_root=$(mktemp -d "$WORK_DIR/package-${SUMO_VERSION}-${platform}.XXXXXX")
package_dir="$stage_root/$platform"
mkdir -p "$package_dir/include/libsumo" "$package_dir/include/utils/geom" \
    "$package_dir/lib" "$package_dir/licenses/sumo" \
    "$package_dir/licenses/xerces-c" "$package_dir/licenses/zlib"

cp -p "$build_root/sumo-$SUMO_VERSION/Release/src/config.h" "$package_dir/include/"
for header in Simulation.h Vehicle.h VehicleType.h TraCIDefs.h TraCIConstants.h; do
    cp -p "$SUMO_SOURCE_DIR/src/libsumo/$header" "$package_dir/include/libsumo/"
done
for header in PositionVector.h AbstractPoly.h Position.h; do
    cp -p "$SUMO_SOURCE_DIR/src/utils/geom/$header" "$package_dir/include/utils/geom/"
done
cp -p "$SUMO_SOURCE_DIR/LICENSE" "$SUMO_SOURCE_DIR/NOTICE.md" "$package_dir/licenses/sumo/"
cp -p "$SUMO_SOURCE_DIR/src/foreign/RoutingKit/LICENSE" "$package_dir/licenses/sumo/RoutingKit-LICENSE"
cp -p "$SUMO_SOURCE_DIR/src/foreign/zstr/LICENSE" "$package_dir/licenses/sumo/zstr-LICENSE"
cp -p "$xerces_source/LICENSE" "$xerces_source/NOTICE" "$package_dir/licenses/xerces-c/"
cp -p "$zlib_source/README" "$package_dir/licenses/zlib/"

for configuration in "${configurations[@]}"; do
    sumo_build="$build_root/sumo-$SUMO_VERSION/$configuration"
    suffix=
    [[ "$configuration" != Debug ]] || suffix=d
    sumo_name="libsumostatic.$lib_ext"
    [[ "$configuration" != Debug ]] || sumo_name="libsumostaticD.$lib_ext"
    cp -p "$sumo_build/lib/$sumo_name" "$package_dir/lib/${lib_prefix}libsumostatic${suffix}.$lib_ext"
    for module in "${sumo_modules[@]}"; do
        cp -p "$sumo_build/lib/${lib_prefix}${module}.$lib_ext" \
            "$package_dir/lib/${lib_prefix}${module}${suffix}.$lib_ext"
    done
    zlib_name="${lib_prefix}z.$lib_ext"
    xerces_name="${lib_prefix}xerces-c.$lib_ext"
    if [[ "$platform" == v10 ]]; then
        zlib_name="zlibstatic${suffix}.lib"
        xerces_name=xerces-c_3.lib
        [[ "$configuration" != Debug ]] || xerces_name=xerces-c_3D.lib
    fi
    cp -p "$sumo_build/dependencies/lib/$zlib_name" "$package_dir/lib/${lib_prefix}zlibstatic${suffix}.$lib_ext"
    xerces_suffix=
    [[ "$configuration" != Debug ]] || xerces_suffix=D
    cp -p "$sumo_build/dependencies/lib/$xerces_name" "$package_dir/lib/${lib_prefix}xerces-c_3${xerces_suffix}.$lib_ext"
done

archive="$WORK_DIR/sumo_${platform}_${SUMO_VERSION}.7z"
(
    cd "$stage_root"
    "$SEVEN_ZIP" a -t7z -m0=LZMA2 "-mmt=$PARALLEL_BUILDS" -mx=5 package.7z "$platform"
    "$SEVEN_ZIP" t package.7z
)
mv -f "$stage_root/package.7z" "$archive"
printf '\nPackage: %s\nArchive: %s\n' "$package_dir" "$archive"
