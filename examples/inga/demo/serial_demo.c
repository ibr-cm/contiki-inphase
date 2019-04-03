 #include "contiki.h"
 #include "dev/serial-line.h"
 #include <stdio.h>

 PROCESS(test_serial, "Serial line test process");
 AUTOSTART_PROCESSES(&test_serial);

 PROCESS_THREAD(test_serial, ev, data)
 {
   PROCESS_BEGIN();

   // you can print to the serial terminal via plain printf
   printf("test message\n");

   // you can also print single characters with putchar
   // note the single quotes to indicate single characters
   putchar('t');
   putchar('e');
   putchar('s');
   putchar('t');
   putchar('\n');

   // you can also write character via their ascii code to the terminal
   putchar(0x41); // character A
   putchar(0x0A); // linefeed (\n)

   // serial commands can be read via events
   // Contiki OS sends a special event to all processes every time a new line
   // is received via serial.
   // Attention: you only receive an event when the line is finished with \n!

   // main (infinite) loop
   for(;;) {
     PROCESS_YIELD(); // hand control back to scheduler and wait for events

     // if we reach here, we got a new event
     if(ev == serial_line_event_message) { // check if it is a serial line event
       printf("received line: %s\n", (char *)data); // print the received string
     }
   }
   PROCESS_END();
 }
