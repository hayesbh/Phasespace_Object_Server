/**
 * File: Glove.h
 * Author: Dylan Visher
 * Date: 5/18/13
 * About: Glove extends Object Type adding in more specific "Hand" information
 */
#ifndef _SHL_COREOBJECTSERVER_OBJECTTYPE_H
#define _SHL_COREOBJECTSERVER_GLOVE_H

namespace object_server {

using object_server::ObjectType;

class Glove : public ObjectType {
private:
	int thumb;
	int fore;
	int middle;
	int ring;
	int pinkey;
	int base_left;
	int base_right;

};

} //object_server

#endif