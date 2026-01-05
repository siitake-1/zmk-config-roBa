# SPDX-License-Identifier: MIT

zephyr_library()

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/../app/include
)

zephyr_library_sources_ifdef(CONFIG_ZMK_DISPLAY_STATUS_SCREEN_CUSTOM status_screen.c)




