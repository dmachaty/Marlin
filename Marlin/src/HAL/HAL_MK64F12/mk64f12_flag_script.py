from __future__ import print_function
import sys

#dynamic build flags for generic compile options
if __name__ == "__main__":
  args = " ".join([ "-std=c11",
                    "-std=c++14",
                    "-Os",
                    "-mcpu=cortex-m4",
                    "-mthumb",

                    "-fsigned-char",
                    "-fno-move-loop-invariants",
                    "-fno-strict-aliasing",
                    "-fsingle-precision-constant",

                    "--specs=nano.specs",
                    "--specs=nosys.specs",

                    # For external libraries
                    "-IMarlin/src/HAL/HAL_MK64F12/include",

                    # For MarlinFirmware/U8glib-HAL
                    # TODO - will be allowed later
                    #"-IMarlin/src/HAL/HAL_MK64F12/u8g",
                    #"-DU8G_HAL_LINKS",

                    "-MMD",
                    "-MP",
                    "-DTARGET_MK64FN1M0",
                    "-DCPU_MK64FN1M0VLL12"
                  ])
                  
  for i in range(1, len(sys.argv)):
    args += " " + sys.argv[i]

  print(args)

  # extra script for linker options
else:
  from SCons.Script import DefaultEnvironment
  env = DefaultEnvironment()
  env.Append(
      ARFLAGS=["rcs"],

      ASFLAGS=["-x", "assembler-with-cpp"],

      CXXFLAGS=[
          "-fabi-version=0",
          "-fno-use-cxa-atexit",
          "-fno-threadsafe-statics"
      ],
      LINKFLAGS=[
          "-Wl,-Tframeworks/CMSIS-MK64F12/MK64F12/lib/MK64FN1M0xxx12_flash.ld,--gc-sections",
          "-Os",
          "-mcpu=cortex-m4",
          "-mthumb",
          #"--specs=nano.specs",
          #"--specs=nosys.specs",
          "-u_printf_float"
      ],
  )
