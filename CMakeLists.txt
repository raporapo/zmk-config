# Out-of-tree drivers for the boards/shields in this zmk-config
if(CONFIG_TMAG5273_RING)
  zephyr_library()
  zephyr_library_sources(drivers/input/tmag5273_ring.c)
endif()
