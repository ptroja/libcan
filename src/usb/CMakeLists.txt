remake_include(
  ../config
  ../can
)
remake_add_library(
  usb
  LINK can
)
remake_add_headers()
