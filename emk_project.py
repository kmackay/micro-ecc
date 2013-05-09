c, link = emk.module("c", "link")

default_flags = ["-fno-common", "-Wall", "-Wextra"]
opt_flags = {"prf":["-pg", "-O2"], "dbg":["-g"], "std":["-O1", "-DNDEBUG"], "opt":["-O2", "-DNDEBUG"], "max":["-O3", "-DNDEBUG"]}

opt_level = "dbg"
if "opt" in emk.options:
    level = emk.options["opt"]
    if level in opt_flags:
        opt_level = level
emk.options["opt"] = opt_level

c.flags.extend(default_flags)
c.flags.extend(opt_flags[opt_level])

c.include_dirs.append("$:proj:$")
