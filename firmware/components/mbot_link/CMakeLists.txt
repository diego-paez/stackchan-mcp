# Not built as a standalone component in this project: main/CMakeLists.txt
# pulls these sources into main when CONFIG_STACKCHAN_MBOT_LINK is set
# (MINIMAL_BUILD would otherwise trim the include paths, see feetech_scs).
# Kept so the directory can be reused as a regular ESP-IDF component.
idf_component_register(
    SRCS        "f3_codec.cc" "mbot_command.cc" "mbot_link.cc"
    INCLUDE_DIRS "."
    REQUIRES    bt json nvs_flash esp_timer
)
