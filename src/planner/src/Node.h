#ifndef NODE_H
#define NODE_H

class Node {
public:
    float gcost;
    float hcost;
    int x;
    int y;
    Node* parent;
    bool blocked;

    Node(float g = 0, float h = 0, Node* p = nullptr, bool b = false,int x = -1 ,int y = -1);

    float getFCost();

};

#endif // NODE_H
