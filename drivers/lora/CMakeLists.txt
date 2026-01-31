# SPDX-License-Identifier: Apache-2.0

zephyr_sources_ifdef(CONFIG_LORA_SHELL shell.c)

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_LORA_MODULE_BACKEND_LORAMAC_NODE loramac-node)
add_subdirectory_ifdef(CONFIG_LORA_MODULE_BACKEND_LORA_BASICS_MODEM lora-basics-modem)
add_subdirectory_ifdef(CONFIG_LORA_MODULE_BACKEND_NONE sx126x)
# zephyr-keep-sorted-stop
