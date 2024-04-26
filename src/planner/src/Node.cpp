#include "Node.h"

Node::Node(float g, float h, Node* p, bool b, int x,int y)
    : gcost(g), hcost(h), parent(p), blocked(b) , x(x), y(y){}

float Node::getFCost(){
    return hcost + gcost;
}