# Check if the serial library is available
find_path(serial_INCLUDE_DIRS NAMES serial.h PATHS /usr/local/include/serial)

# Check if the serial library is available
find_library(serial_LIBRARIES NAMES libserial.so PATHS /usr/local/lib)

# Set serial_FOUND to TRUE if both the include directory and library are found
if (serial_INCLUDE_DIRS AND serial_LIBRARIES)
    set(serial_FOUND TRUE)
else ()
    set(serial_FOUND FALSE)
endif ()

# Provide the version of serial if it's available
find_file(serial_VERSION_FILE NAMES version.txt PATHS /usr/local/lib/cmake/serial)
if (serial_VERSION_FILE)
    file(STRINGS ${serial_VERSION_FILE} serial_VERSION)
endif ()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(serial DEFAULT_MSG serial_LIBRARIES serial_INCLUDE_DIRS)

if (serial_FOUND)
    # Include directories for the serial library
    set(serial_INCLUDE_DIRS ${serial_INCLUDE_DIRS})

    # Library file for serial
    set(serial_LIBRARIES ${serial_LIBRARIES})
endif ()
