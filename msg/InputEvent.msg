string MOUSE_MOVE = move
string MOUSE_PRESS = press
string MOUSE_RELEASE = release
string DOUBLE_CLICK = dbclick
string MOUSE_LEAVE = leave
string MOUSE_ENTER = enter

uint8 LEFT_BUTTON = 1
uint8 RIGHT_BUTTON = 2
uint8 MIDDLE_BUTTON = 4

Header header

string event_type
uint8 event_button
uint8 buttons_pressed

Point2i window_point
Point2d image_point

Size2i window_size
Size2i image_size

string image_topic
string window_name

time image_stamp