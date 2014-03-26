import os

c, link = emk.module("c", "link")

default_compile_flags = ["-fvisibility=hidden", "-Wall", "-Wextra", "-Wshadow", "-Werror", "-Wno-missing-field-initializers", "-Wno-unused-parameter", \
    "-Wno-comment", "-Wno-unused", "-Wno-unknown-pragmas"]
default_link_flags = []
opt_flags = {"dbg":["-g"], "std":["-O2"], "max":["-O3"], "small":["-Os"]}
opt_link_flags = {"dbg":[], "std":[], "max":[], "small":[]}
c_flags = ["-std=c99"]
cxx_flags = ["-std=c++11", "-Wno-reorder", "-fno-rtti", "-fno-exceptions"]
c_link_flags = []
cxx_link_flags = ["-fno-rtti", "-fno-exceptions"]

def setup_build_dir():
    build_arch = None
    if "arch" in emk.options:
        build_arch = emk.options["arch"]
    elif not emk.cleaning:
        build_arch = "osx"
    emk.options["arch"] = build_arch

    opt_level = None
    if "opt" in emk.options:
        level = emk.options["opt"]
        if level in opt_flags:
            opt_level = level
        else:
            emk.log.warning("Unknown optimization level '%s'" % (level))
    elif not emk.cleaning:
        opt_level = "dbg"
    emk.options["opt"] = opt_level

    dirs = ["__build__"]
    if build_arch:
        dirs.append(build_arch)
    if opt_level:
        dirs.append(opt_level)
    emk.build_dir = os.path.join(*dirs)

def setup_osx():
    global c
    global link

    flags = [("-arch", "x86_64"), "-fno-common", "-Wnewline-eof", "-mmacosx-version-min=10.7", "-D__DARWIN_UNIX03=0"]
    c.flags.extend(flags)
    c.cxx.flags += ["-stdlib=libc++"]
    link.cxx.flags += ["-stdlib=libc++"]

    link_flags = [("-arch", "x86_64"), "-mmacosx-version-min=10.7"]
    link.local_flags.extend(link_flags)

    dylib_flags = [("-current_version", "1.0"), ("-compatibility_version", "1.0")]
    link.local_libflags.extend(dylib_flags)

def setup_avr():
    global c
    global link

    c.compiler = c.GccCompiler("/Projects/avr-tools/bin/avr-")
    c.flags += ["-mmcu=atmega256rfr2", "-ffunction-sections", "-fdata-sections"]
    link.linker = link.GccLinker("/Projects/avr-tools/bin/avr-")
    link.flags += ["-mmcu=atmega256rfr2", "-mrelax", "-Wl,--gc-sections"]
    link.strip = True

setup_build_dir()

setup_funcs = {"osx":setup_osx, "avr":setup_avr}

if not emk.cleaning:
    build_arch = emk.options["arch"]
    opt_level = emk.options["opt"]

    c.flags.extend(default_compile_flags)
    c.flags.extend(opt_flags[opt_level])
    c.c.flags.extend(c_flags)
    c.cxx.flags.extend(cxx_flags)
    link.local_flags.extend(default_link_flags)
    link.local_flags.extend(opt_link_flags[opt_level])
    link.c.local_flags.extend(c_link_flags)
    link.cxx.local_flags.extend(cxx_link_flags)

    c.include_dirs.append("$:proj:$")

    if build_arch in setup_funcs:
        setup_funcs[build_arch]()
    else:
        raise emk.BuildError("Unknown target arch '%s'" % (build_arch))

    c.defines["TARGET_ARCH_" + build_arch.upper()] = 1
