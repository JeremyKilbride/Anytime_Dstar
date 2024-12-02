#ifndef PLANNING_PROJECT_ANYTIME_D_STAR_H
#define PLANNING_PROJECT_ANYTIME_D_STAR_H


#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <queue>
#include <limits>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>
#ifndef MAPS_DIR
#define MAPS_DIR "maps"
#endif
#define SENSOR_REACH 10
using std::cout;


int get_key(int x_size, int x, int y){ //IF MAPS ARE 0 indexed
    return y*x_size + x;
}

struct Node
{
    int x;
    int y;
    double g=std::numeric_limits<double>::max();
    double h=std::numeric_limits<double>::max();
    double v=std::numeric_limits<double>::max();
    int parent_idx=-1;

    Node(int x, int y): x(x), y(y){}

    friend bool operator>(Node& lhs, Node& rhs);

    Node(){}

    void compute_h(){//TODO:update this
        this->h=0;
    }

    Node operator=(const Node& incoming){
        this->x=incoming.x;
        this->y=incoming.y;
        this->g=incoming.g;
        this->h=incoming.h;
        this->v=incoming.v;
        this->parent_idx=incoming.parent_idx;
        return *this;
    }
};

int get_key(int x_size, Node current){ //IF MAPS ARE 0 indexed
    return current.y*x_size + current.x;
};
int get_key(int x_size, std::shared_ptr<Node> current){
    return current->y*x_size + current->x;
};

using NodePtr=std::shared_ptr<Node>;

class Graph
{
    private:
        std::unordered_map<int, Node> nodeMap;
        int x_size;
        int y_size;
        int start_idx;
    public:
        Graph(int x_size, int y_size):x_size(x_size),y_size(y_size){
            nodeMap.reserve(x_size*y_size);
        }
    
        void set_start(int s){this->start_idx=s;}    
    
        void addNode(Node& newNode){
            int idx=get_key(x_size,newNode.x,newNode.y);
            nodeMap.emplace(idx,newNode);
        }
        

        void addNode(int x, int y){
            Node newNode(x,y);
            nodeMap.emplace(get_key(x_size,newNode.x,newNode.y),newNode);
        }

     
        int _x_size(){return x_size;}

        int _y_size(){return y_size;}
        
        std::shared_ptr<Node> getAddNode(int idx){
            auto it=nodeMap.find(idx);
            if (it!=nodeMap.end()){
                return std::make_shared<Node>(it->second);
            }
            else{
                int new_x=idx%x_size; 
                int new_y=idx/x_size; 
                Node _newNode(new_x,new_y);
                _newNode.compute_h();
                addNode(_newNode);
                auto it=nodeMap.find(idx);
                if (it==nodeMap.end()){
                    return nullptr;
                }
                else
                    return std::make_shared<Node>(it->second);
            }
        }

        void set(Node& s){
            int idx=get_key(x_size,s.x,s.y);
            auto it=nodeMap.find(idx);
            if (it==nodeMap.end()){
                cout<<"something went wrong with setting\n";
                return;
            }
            nodeMap[idx]=s;
        }

        std::shared_ptr<Node> get(Node& newNode){
            int idx=get_key(x_size,newNode.x,newNode.y);
            auto it = nodeMap.find(idx);
            if(it==nodeMap.end()){
                return nullptr;
            }else{
                return std::make_shared<Node>(it->second);
            }
        }
        std::shared_ptr<Node> get(int x, int y){
            auto it = nodeMap.find(get_key(x_size,x,y));
            if(it==nodeMap.end()){
                return nullptr;
            }else{
                return std::make_shared<Node>(it->second);
            }
        }


        void print()
        {
            for (auto item: nodeMap){
                cout<<"key: "<<item.first<<"\n";
            }
        }

        std::shared_ptr<Node> get(int idx){
            auto it = nodeMap.find(idx);
            if(it==nodeMap.end()){
                return nullptr;
            }
            else
                return std::make_shared<Node>(it->second);
        }

};




std::unordered_set<int> update_map(int*& current,int* global, int x_size, int y_size, int robotposeX, int robotposeY){
    std::unordered_set<int> cell_changes;
    
    for(int i =0; i< x_size;i++){
        for(int j=0; j<y_size;j++){
            if(i>robotposeX-SENSOR_REACH && i<robotposeX+SENSOR_REACH){
                if(j>robotposeY-SENSOR_REACH && j<robotposeY+SENSOR_REACH){
                    if(current[get_key(x_size,i,j)] != global[get_key(x_size,i,j)]){
                        current[get_key(x_size,i,j)] = global[get_key(x_size,i,j)];
                        cell_changes.insert(get_key(x_size,i,j));
                    }
                    
                }
            }
        }

    }
    return cell_changes;
}

bool read_map(std::string map_name, int*& map, int& x_size, int& y_size, Node& start_node, Node& goal_node){
    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + map_name;

    std::ifstream myfile;
    myfile.open(mapFilePath);
    if (!myfile.is_open()) {
        std::cout << "Failed to open the file:" << mapFilePath << std::endl;
        return false;
    }

    // read map size
    char letter;
    std::string line;
    
    myfile >> letter;

    if (letter != 'N')
    {
        std::cout << "error parsing file" << std::endl;
        return false;
    }
    myfile >> x_size >> letter >> y_size;
    std:: cout << "map size: " << x_size << letter << y_size << std::endl;

    // read robot position
    int robotposeX, robotposeY;
    myfile >> letter;
    if (letter != 'R')
    {
        std::cout << "error parsing file" << std::endl;
        return false;
    }

    myfile >> robotposeX >> letter >> robotposeY;
    start_node.x = robotposeX;
    start_node.y =robotposeY;

    myfile >> letter;

    if (letter != 'G')
    {
        std::cout << "error parsing file" << std::endl;
        return false;
    }
    int goalposeX, goalposeY;
    myfile >> goalposeX >> letter >> goalposeY;

    goal_node.x = goalposeX;
    goal_node.y = goalposeY;


    myfile >> letter;
    if (letter != 'M')
    {
        std::cout << "error parsing file" << std::endl;
        return false;
    }


    myfile.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    map = new int[x_size*y_size];
    for (size_t i=0; i<x_size; i++)
    {
        std::getline(myfile, line);
        std::stringstream ss(line);
        for (size_t j=0; j<y_size; j++)
        {
            double value;
            ss >> value;
            map[j*x_size+i] = (int) value;
            if (j != y_size-1) ss.ignore();
        }
    }
    myfile.close();
    return true;

};

#endif
