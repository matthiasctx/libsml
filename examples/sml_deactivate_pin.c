// Copyright 2011 Juri Glass, Mathias Runge, Nadim El Sayed
// DAI-Labor, TU-Berlin
//
// This file is part of libSML.
//
// libSML is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libSML is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libSML.  If not, see <http://www.gnu.org/licenses/>.

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include <sml/sml_file.h>
#include <sml/sml_transport.h>
#include <sml/sml_open_request.h>
#include <sml/sml_close_request.h>
#include <sml/sml_tree.h>
#include <sml/sml_octet_string.h>
#include <sml/sml_value.h>
#include <sml/sml_boolean.h>
#include <sml/sml_set_proc_parameter_request.h>



int serial_port_open(const char* device) {
    int bits;
    struct termios config;
    memset(&config, 0, sizeof(config));

    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        printf("error: open(%s): %s\n", device, strerror(errno));
        return -1;
    }

    // set RTS
    ioctl(fd, TIOCMGET, &bits);
    bits |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &bits);

    tcgetattr( fd, &config ) ;

    // set 8-N-1
    config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    config.c_oflag &= ~OPOST;
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    config.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB);
    config.c_cflag |= CS8;

    // set speed to 9600 baud
    cfsetispeed( &config, B9600);
    cfsetospeed( &config, B9600);

    tcsetattr(fd, TCSANOW, &config);
    return fd;
}

void transport_receiver(unsigned char *buffer, size_t buffer_len) {
    // the buffer contains the whole message, with transport escape sequences.
    // these escape sequences are stripped here.
    sml_file *file = sml_file_parse(buffer + 8, buffer_len - 16);
    // the sml file is parsed now

    // read here some values ..

    // this prints some information about the file
    sml_file_print(file);

    // free the malloc'd memory
    sml_file_free(file);
}

int main(int argc, char **argv) {

	// This code is sending a SML Set Proc Parameter Request to the smart meter 
	// in order to deactivate the PIN code that protects some values in the smart meters display.
	// It currently works with eHZ meters of EMH metering GmbH
	// To make this work you need a serial IR transmitter connected to a serial port
	// The IR transmitter must use the REAR IR interface of the smart meter. It does
	// NOT work with the front IR interface as this interface is read only.
	// Note: usually you cannot attach the IR transmitter to the back of your
	// smart meter without breaking the seals that your energy provider attached to it.
	// Breaking the seals is against the law in Germany!

	// Initialize strings for client ID and request file ID
    octet_string *client_id = sml_octet_string_init((unsigned char *)"smlclient", 9);
    octet_string *req_file_id = sml_octet_string_init((unsigned char *)"smlreqfileid", 12);

    //Create a SML Open Request
    sml_open_request *or = sml_open_request_init();
    or->client_id = client_id;
    or->req_file_id = req_file_id;

    // Create a SML Close Request
    sml_close_request *cl = sml_close_request_init();

    // Create a boolean value and set it to false
    sml_boolean *smlb = sml_boolean_init(SML_BOOLEAN_FALSE);
    sml_value *value = sml_value_init();
    value->type = SML_TYPE_BOOLEAN;
    value->data.boolean = smlb;

    // Create a proc parameter value and assign the boolean value
    sml_proc_par_value *ppv = sml_proc_par_value_init();
    u8 ppvtag = SML_PROC_PAR_VALUE_TAG_VALUE;
    ppv->tag = &ppvtag;
    ppv->data.value = value;

    // Create a sml tree and set parameter name and value
    // The parameter name is the OBIS value to configure PIN code protection
    sml_tree *tree = sml_tree_init();
    unsigned char parameterName[6] = {0x81,0x81,0xc7,0x8c,0x0a,0xff};
    tree->parameter_name = sml_octet_string_init(parameterName, 6);
    tree->parameter_value = ppv;
    tree->child_list = NULL;
    // We also need a tree path referencing our tree entry by the parameter name
    sml_tree_path *tp = sml_tree_path_init();
    sml_tree_path_add_path_entry(tp, sml_octet_string_init(parameterName, 6));

    // The final set proc parameter request. Assigning tree and tree path
    sml_set_proc_parameter_request *sppr = sml_set_proc_parameter_request_init();
    sppr->parameter_tree_path = tp;
    sppr->parameter_tree = tree;

    // Our group id and intended on error behaviour. See SML spec for details.
    u8 group_id = 1;
    u8 abort_on_error = 0;

    // Now we need to create SML messages from all the requests we just created
    // The SML message gets a type according to the request and then we assign 
    // group id, abort behaviour and the request
    sml_message *openmsg = sml_message_init();
    sml_message_body *openmsgbody = sml_message_body_init(SML_MESSAGE_OPEN_REQUEST, or);
    openmsg->group_id = &group_id;
    openmsg->abort_on_error = &abort_on_error;
    openmsg->message_body = openmsgbody;

    // The close request message
    sml_message *closemsg = sml_message_init();
    sml_message_body *closemsgbody = sml_message_body_init(SML_MESSAGE_CLOSE_REQUEST, cl);
    closemsg->group_id = &group_id;
    closemsg->abort_on_error = &abort_on_error;
    closemsg->message_body = closemsgbody;

    // The set proc parameter request message
    sml_message *spprmsg = sml_message_init();
    sml_message_body *spprmsgbody = sml_message_body_init(SML_MESSAGE_SET_PROC_PARAMETER_REQUEST, sppr);
    spprmsg->group_id = &group_id;
    spprmsg->abort_on_error = &abort_on_error;
    spprmsg->message_body = spprmsgbody;

    // In order to send the messages to the smart meter we need to pack them into a SML file
    // As per SML spec a file always consists of an open request at the beginning and a close request at the end
    // In between we add our set proc parameter request
    sml_file *smlf = sml_file_init();
    sml_file_add_message(smlf, openmsg);
    sml_file_add_message(smlf, spprmsg);
    sml_file_add_message(smlf, closemsg);

    // printing, just to make sure that our file can be parsed correctly
    sml_file_print(smlf);


    // this example assumes that a EDL21 meter sending SML messages via a
    // serial device. Adjust as needed.
    char *device = "/dev/ttyUSB0";
    int fd = serial_port_open(device);

    if (fd > 0) {
    	// Write the SML file to serial port
        sml_transport_write(fd, smlf);
        // listen on the serial device, this call is blocking.
        // The smart meter should resond with an attention response 0xFF01
        sml_transport_listen(fd, &transport_receiver);
        close(fd);
    }

    return 0;
}
