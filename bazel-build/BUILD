cc_binary(
    name = "roboclaw",
    srcs = ["roboclaw.cpp"],
    deps = [
        ":libserial",
    ],
    linkopts=['-static'],
    linkstatic = True,  # Ensure static linking
)

cc_library(
    name = "libserial",
    srcs = ["libserial/build/lib/libserial.a"],
    hdrs = glob(["libserial/include/*.h"]),
    includes = ["libserial/include"],
    linkopts=['-static'],
    linkstatic = True,  # Ensure static linking
)
