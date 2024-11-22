#ifndef PLANNING_PROJECT_ANYTIME_D_STAR_H
#define PLANNING_PROJECT_ANYTIME_D_STAR_H


#include <unordered_map>
#include <iostream>
#include <queue>
#include <limits>
#include <memory>
#include <fstream>
#include <sstream>
#ifndef MAPS_DIR
#define MAPS_DIR = "maps"
#endif

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

    Node(){}
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

void read_map(std::string map_name, int*& map, int& x_size, int& y_size, Node& start_node, Node& goal_node){
    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + map_name;

    std::ifstream myfile;
    myfile.open(mapFilePath);
    if (!myfile.is_open()) {
        std::cout << "Failed to open the file:" << mapFilePath << std::endl;
        return;
    }

    // read map size
    char letter;
    std::string line;
    
    myfile >> letter;

    if (letter != 'N')
    {
        std::cout << "error parsing file" << std::endl;
        return;
    }
    myfile >> x_size >> letter >> y_size;
    std:: cout << "map size: " << x_size << letter << y_size << std::endl;

    // read robot position
    int robotposeX, robotposeY;
    myfile >> letter;
    if (letter != 'R')
    {
        std::cout << "error parsing file" << std::endl;
        return;
    }

    myfile >> robotposeX >> letter >> robotposeY;
    start_node.x = robotposeX;
    start_node.y =robotposeY;

    myfile >> letter;

    if (letter != 'G')
    {
        std::cout << "error parsing file" << std::endl;
        return;
    }
    int goalposeX, goalposeY;
    myfile >> goalposeX >> letter >> goalposeY;

    goal_node.x = goalposeX;
    goal_node.y = goalposeY;


    myfile >> letter;
    if (letter != 'M')
    {
        std::cout << "error parsing file" << std::endl;
        return;
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

};

#endif