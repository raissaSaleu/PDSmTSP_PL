#ifndef _Node_h
#define _Node_h
#include <fstream>
#include <vector>
#include <deque>



using namespace std;


class Node
{
    public:
        Node(int idin, double xin, double yin, int visit, int drone);
        Node(Node & source);
        Node ();
        ~Node();// Destructor

        void set_x(double i); //Set the value of x variable
        double get_x(); //Returns the value of x variable

        void set_y(double i); //Set the value of y variable
        double get_y(); //Returns the value of y variable

        void set_id(int i); //Set the value of id variable
        int get_id(); //Returns the value of id variable

        void set_drone(int i); //Set to 0 if the customer is drone eligible and 1 else
        int get_drone(); //Returns the value of drone variable

    private:
        int id;
        double x;
        double y;
        int drone;


};

#endif
