#ifndef NN_ICP
#define NN_ICP

#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class Kdnode
{
public:
    Vector2f data;
    Kdnode *left = NULL;
    Kdnode *right= NULL;

    void print_kdnode()
    {
        cout << "classdata: " << data(0) << " " << data(1) << endl;
        if (left)
        {
            left->print_kdnode();
        }
        if (right)
        {
            right->print_kdnode();
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void print_point(const vector<Vector2f> &reference_point)
{
    for (int i = 0; i < reference_point.size(); i++)
    {
        cout << reference_point[i](0) << " " << reference_point[i](1) << endl;
    }
    cout << "end" << endl;
    return;
}

void kd_tree(vector<Vector2f> &reference_point, const int depth, Kdnode *current)
{
    int index = reference_point.size();
    int axis = depth % 2;
    if (index == 0)
    {
        //delete current;
        return;
    }

    if (axis == 0)
    {
        sort(reference_point.begin(), reference_point.end(), [&](const Vector2f &a, const Vector2f &b) -> bool
             { return a(0) < b(0); });
    }
    else
    {
        sort(reference_point.begin(), reference_point.end(), [&](const Vector2f &a, const Vector2f &b) -> bool
             { return a(1) < b(1); });
    }
    int median = index / 2;
    current->data = reference_point[median];

    if (median == 0)
    {
        return;
    }

    // if (index == 2)
    // {
    //     vector<Vector2f> temp1 = vector<Vector2f>(reference_point.begin(), reference_point.begin() + median);

    //     Kdnode *left = new Kdnode;

    //     current->left = left;

    //     kd_tree(temp1, depth + 1, left);
    // }
    // else
    // {
        vector<Vector2f> temp1 = vector<Vector2f>(reference_point.begin(), reference_point.begin() + median);
        vector<Vector2f> temp2 = vector<Vector2f>(reference_point.begin() + median + 1, reference_point.end());

        Kdnode *left = new Kdnode;
        Kdnode *right = new Kdnode;

    if (temp1.size()){
        current->left = left;
        kd_tree(temp1, depth + 1, left);
    }
    if (temp2.size()){
        current->right = right;
        kd_tree(temp2, depth + 1, right);
    }
        

        
        
    //}
}

Kdnode *closest(Kdnode *A, Kdnode *B, const Vector2f &point)
{
    if (!A)
        return B;

    if (!B)
        return A;

    double d1 = (A->data - point).transpose() * (A->data - point);
    double d2 = (B->data - point).transpose() * (B->data - point);

    if (d1 < d2)
        return A;
    else
        return B;
}

Kdnode *nn(Kdnode *current, const Vector2f &point, int depth)
{
    if (!current)
        return NULL;

    Kdnode *next_b, *other_b;
    
    if (point(depth % 2) < current->data(depth % 2))
    {
        next_b = current->left;
        other_b = current->right;
    }
    else
    {
        next_b = current->right;
        other_b = current->left;
    }

    Kdnode *temp = nn(next_b, point, depth + 1);
    Kdnode *best = closest(temp, current, point);
    
    double radius = (best->data - point).transpose() * (best->data - point);
    double dist = point(depth % 2) - current->data(depth % 2);

    if (radius >= dist * dist)
    {
        Kdnode *temp = nn(other_b, point, depth + 1);
        Kdnode *best = closest(temp, best, point);
    }
    return best;
}
/*
int main()
{
    Kdnode *root = new Kdnode;

    vector<Vector2f> test;
    Vector2f L1, L2, L3, L4, L5, L6, test_point;

    L1 << 7.5, 2.0;
    L2 << 5, 4;
    L3 << 9.4, 6.5;
    L4 << 2, 3;
    L5 << 4, 7;
    L6 << 8, 1;
    test_point << 9.1, 9.6;

    test.push_back(L1);
    test.push_back(L2);
    test.push_back(L3);
    test.push_back(L4);
    test.push_back(L5);
    test.push_back(L6);

    L1 << 4.44, 21.1;
    L2 << 0, 2;
    L3 << 2.4, 35;
    L4 << 3, 4;
    L5 << 6, 10;
    L6 << 4, 14;

    test.push_back(L1);
    test.push_back(L2);
    test.push_back(L3);
    test.push_back(L4);
    test.push_back(L5);
    test.push_back(L6);

    kd_tree(test, 0, root);

    root->print_kdnode();

    

    for (int i = 0; i< 500;i++){
        Kdnode *best = nn(root,test[i % 12],0);
    }

    return 0;
}
*/
#endif
