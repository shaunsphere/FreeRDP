# FreeRDP: A Remote Desktop Protocol Implementation
# FreeRDP cmake build script
#
# Copyright 2012 Atrust corp.
# Copyright 2012 Alfred Liu <alfred.liu@atruscorp.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

define_channel_client_subsystem("urbdrc" "libusb" "")

find_package(libusb-1.0)
if (NOT LIBUSB_1_FOUND)
    message(FATAL_ERROR "libusb-1 missing!")
endif()

set(${MODULE_PREFIX}_SRCS
	libusb_udevman.c
	libusb_udevice.c
	libusb_udevice.h
	request_queue.c
	request_queue.h)

include_directories(..)
include_directories(${LIBUSB_1_INCLUDE_DIRS})

add_channel_client_subsystem_library(${MODULE_PREFIX} ${MODULE_NAME} ${CHANNEL_NAME} "" TRUE "")

set(${MODULE_PREFIX}_LIBS
	${CMAKE_THREAD_LIBS_INIT}
        ${DBUS_GLIB_LIBRARIES}
	${LIBUSB_1_LIBRARIES}
	winpr)

target_link_libraries(${MODULE_NAME} ${${MODULE_PREFIX}_LIBS})

install(TARGETS ${MODULE_NAME} DESTINATION ${FREERDP_ADDIN_PATH} EXPORT FreeRDPTargets)

