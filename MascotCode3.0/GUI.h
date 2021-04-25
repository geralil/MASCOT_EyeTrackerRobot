//#include <Nextion.h>
//
//NexPage page0 = NexPage(0, 0, "Menu");
//NexPage page1 = NexPage(1, 0, "Neck");
//NexPage page2 = NexPage(2, 0, "Gaze");
//NexPage page3 = NexPage(3, 0, "Shoulder");
//
//int currentPage = 0;
//
//NexTouch *nex_listen_list[] = 
//{
//  &page0,
//  &page1,
//  &page2,
//  NULL
//};    // End of touch event list
//
//
//// Page change event:
//void page0PushCallback(void *ptr)  // If page 0 is loaded on the display, the following is going to execute:
//{
//  CurrentPage = 0;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
//}  // End of press event
//
//
//// Page change event:
//void page1PushCallback(void *ptr)  // If page 1 is loaded on the display, the following is going to execute:
//{
//  CurrentPage = 1;  // Set variable as 1 so from now on arduino knows page 1 is loaded on the display
//}  // End of press event
//
//
//// Page change event:
//void page2PushCallback(void *ptr)  // If page 2 is loaded on the display, the following is going to execute:
//{
//  CurrentPage = 2;  // Set variable as 2 so from now on arduino knows page 2 is loaded on the display
//}  // End of press event
//
//// Page change event:
//void page3PushCallback(void *ptr)  // If page 2 is loaded on the display, the following is going to execute:
//{
//  CurrentPage = 3;  // Set variable as 2 so from now on arduino knows page 2 is loaded on the display
//}  // End of press event
//
//
//void WriteInformationToScreen()
//{
//  String sendThis = "";
//  if (currentPage == 0)
//  {
//    sendThis.concat("stateText.txt=Current State: ");
//    sendThis.concat(state);
//    WriteString(sendThis);
//  }
//}
//
//
//void WriteString(String stringData) { // Used to serially push out a String with Serial.write()
//{
//  for (int i = 0; i < stringData.length(); i++)
//  {
//     // Push each char 1 by 1 on each loop pass  
//    mySerial.write(stringData[i]); 
//  }
//
//  mySerial.write(0xff); 
//  mySerial.write(0xff); 
//  mySerial.write(0xff);
//
//  }
//}
