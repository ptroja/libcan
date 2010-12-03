/*
Copyright 2008 Ralf Kaestner <ralf.kaestner@gmail.com>
ASL-ETHZ http://www.asl.ethz.ch/

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The name of the author may not be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <string.h>

#include "can.h"

const char* can_errors[] = {
  "success",
  "error opening CAN device",
  "error setting CAN device parameters",
  "error closing CAN device",
  "error sending CAN message",
  "error receiving CAN message",
};

void can_init(can_device_p dev, config_p config) {
  dev->comm_dev = 0;

  config_init_default(&dev->config, &can_default_config);
  if (config)
    config_set(&dev->config, config);

  dev->num_references = 0;
  dev->num_sent = 0;
  dev->num_received = 0;
}

void can_init_arg(can_device_p dev, int argc, char **argv, const char* 
  prefix) {
  config_t config;
  config_init_arg(&config, argc, argv, (prefix) ? prefix : 
    CAN_CONFIG_ARG_PREFIX);

  can_init(dev, &config);

  config_destroy(&config);
}

void can_destroy(can_device_p dev) {
  config_destroy(&dev->config);
}
