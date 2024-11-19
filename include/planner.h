#ifndef PLANNING_PROJECT_ANYTIME_D_STAR_H
#define PLANNING_PROJECT_ANYTIME_D_STAR_H

#include <unordered_map>
#include <queue>
#include <limits>
#include <memory>

int get_key(int x_size, int y_size, int x, int y){ //IF MAPS ARE 0 indexed
    return y*x_size + x;
};

struct Node
{
    int x;
    int y;
    double g=std::numeric_limits<double>::max();
    double h=std::numeric_limits<double>::max();;
    double v=std::numeric_limits<double>::max();;
    Node(int x, int y): x(x), y(y){}
};

class Graph
{
    private:
        std::unordered_map<int, Node> nodeMap;
        int x_size;
        int y_size;
    public:
        Graph(int x_size, int y_size):x_size(x_size),y_size(y_size){}
        
        void addNode(Node newNode){
            nodeMap.emplace(get_key(x_size,y_size,newNode.x,newNode.y),newNode);
        }

        std::shared_ptr<Node> get(Node newNode){
            auto it = nodeMap.find(get_key(x_size,y_size,newNode.x,newNode.y));
            if(it==nodeMap.end()){
                return nullptr;
            }else{
                return std::make_shared<Node>(it->second);
            }
        }

};

#endif