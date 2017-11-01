#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	for(int j = 0; j < 10; j++) {
	    double alpha = this->ptrTable[i]->alpha;
	    double d = this->ptrTable[i]->d;
	    double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	    double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	    double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	for(int j = 0; j < 10; j++) {
	    double alpha = path[i].alpha;
	    double d = path[i].d;
	    double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	    double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	    double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	}
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    //TODO

    // First, create a new node containing all data
    node *newNode = new node;
    newNode->idx = count;
    newNode->idx_parent = idx_near;
    newNode->alpha = alpha;
    newNode->d = d;
    newNode->location = x_new;
    newNode->rand = x_rand;

    // Then link it to the table of nodes of the RRT Tree
    ptrTable[count] = newNode;
    count++;
}


int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO
    
    // Initialisation was done at construction of the object
    
    // Loop at least K times and until the tree reach the goal
    int loop = 0;
    bool goalReached = false;
    while(loop < K && !goalReached)
    {
        // First step : Generate a random point
        point randPoint = randomState(x_max, x_min, y_max, y_min);
        
        // Get the nearest neighbor
        int nearIndex = nearestNeighbor(randPoint, MaxStep);
        
        // Create a new state
        double out[5];
        if(newState(out, ptrTable[nearIndex]->location, randPoint, MaxStep) == 0)
        {
            // The new state is not valid, we collided with something !
            // So forget about this state, and start a new one, without 
            // incrementing loop, because we want K vertex
            continue;
        }
        
        // The new state is valid ! Create a vertex
        point *newPoint = new point();
        newPoint->x = out[0];
        newPoint->y = out[1];
        newPoint->th = out[2];
        addVertex(*newPoint, randPoint, nearIndex, out[3], out[4]);
        
        // Check if we reached the goal
        if(!goalReached)
        {
            for(int i = 0; i < count; i++)
            {
                // If the goal is 10cm away from one of the point of the tree,
                // it's fine, we accept
                if(distance(ptrTable[i]->location, x_goal) < 0.1)
                {
                    goalReached = true;
                }
            }    
        }
        
        loop++;
        
        // DEBUG
        visualizeTree();
    }
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    //TODO
    
    do
    {
        // Generate random coordinate within the range asked
        float randX = x_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(x_max-x_min)));
        float randY = y_min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(y_max-x_min)));
    
        // Add it to a point
        point random;
        random.x = randX;
        random.y = randY;
        random.th = 0.0;    // shouldn't be used, but we initialise it still.
    }while(isCollision(random, random, 0, max_alpha));
    // Keep generating until we got a valid point
    // We don't care of alpha and d since start and endpoint are the same
    
    return random;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
    
    // TODO : Compute the function f(MaxStep, L, MaxAlpha)
    float thetaMax = max_alpha;

    // Then, go through all the points, minimising the distance
    // and respecting the rule of thetaMax
    node *nearest = root;
    float bestDist = distance(root->location, x_rand);

    // Find the min distance with all the nodes of the tree
    for(i = 0; i < count; i++)
    {
        if(distance(ptrTable[i]->location, x_rand) < bestDist)
        {
            // New minimum distance, but save it only if the angle is respecting
            // the condition for thetaMax
            angle = getDirection(ptrTable[i]->location, x_rand) - ptrTable[i]->alpha;
            if(angle <= thetaMax && angle >= -thetaMax)
            {
                nearest = ptrTable[i];
                bestDist = distance(ptrTable[i]->location, x_rand);
            }
        }
    }

    // Return the index of the nearest point
    return nearest->idx; 
}

int rrtTree::nearestNeighbor(point x_rand) {
    //TODO

    // Initialisation with the root of the tree
    node *nearest = root;
    float bestDist = distance(root->location, x_rand);

    // Find the min distance with all the nodes of the tree
    for(i = 0; i < count; i++)
    {
        if(distance(ptrTable[i]->location, x_rand) < bestDist)
        {
            // New minimum, save it
            nearest = ptrTable[i];
            bestDist = distance(ptrTable[i]->location, x_rand);
        }
    }

    // Return the index of the nearest point
    return nearest->idx;    
}

int rrtTree::newState(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO
    
    // Struct to store the new states randomly generated
    std::vector<*point> randNewPoints;
    
    // Struct to store the control generated with the point
    std::vector<*control> randNewControls;
    
    float xc, yc;
    
    // Generate, let's say, 20 new states
    for(i = 0; i < 20; i++)
    {
        // Generate parameters
        float alpha = -max_alpha + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(max_alpha+max_alpha)));
        float d = static_cast<float>(rand()) / (static_cast<float>(RAND_MAX/(MaxStep)));
        
        // Compute new point with these parameters
        float R = L / tan(alpha);
        float beta = d / R;
        xc = x_near.x - R * sin(x_near.th);
        yc = x_near.y + R * cos(x_near.th); 
        
        point *xNew = new point();
        control *cNew = new control();
        
        xNew->th = x_near.th + beta;
        xNew->x = xc + R * sin(xNew->th);
        xNew->y = xc - R * cos(xNew->th);
        
        cNew->alpha = alpha;
        cNew->d = d;
        
        // Add it to our list of randomly generated point/control
        randNewPoints.push_back(xNew);
        randNewControls.push_back(cNew);
        
        xNew = NULL;
        cNew = NULL;
    }
    
    // We have generated them. Now choose the closest
    std::vector<*point>::iterator itP = randNewPoints.begin();
    std::vector<*control>::iterator itC = randNewControls.begin();
    
    point *bestPoint = randNewPoints.begin();
    control *bestControl = randNewControls.begin();
    float bestDistance = distance(x_rand, *bestPoint);
    
    for(itP = randNewPoints.begin(); itP != randNewPoints.end(); itP++)
    {
        if(distance(x_rand, *itP) < bestDistance)
        {
            bestPoint = itP;
            bestControl = itC;
            bestDistance = distance(x_rand, *bestPoint);
        }
        itC++;
    }
    
    // Now we have the closest point among the 20 randomly generated. 
    // So set the output.
    out[0] = bestPoint->x;
    out[1] = bestPoint->y;
    out[2] = bestPoint->th;
    out[3] = bestControl->alpha;
    out[4] = bestControl->d;
    
    // Return of the function : if it collides or not
    if(isCollision(x_near, *bestPoint, bestControl->d, bestControl->alpha))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

bool rrtTree::isCollision(point x1, point x2, double d, double alpha) {
    //TODO

    // First of all, set up a structure to put all the area of the map 
    // we have to check
    std::vector<int> i;
    std::vector<int> j;
    int iPrime, jPrime;

    // Put the starting point 
    i.push_back(((int)floor(x1.x / res)) + map_origin_x);
    j.push_back(((int)floor(x1.y / res)) + map_origin_y);
    
    // Compute the data we need to compute every point of the path
    float R = L / tan(alpha);
    float beta = d / R;
    
    point xc;
    point xPrime;
    xPrime.x = x1.x;    // Starting point
    xPrime.y = x1.y;
    xPrime.th = x1.th;

    // Then, compute all the way from point x1 to point x2
    while(distance(xPrime, x2) > 0.2)   // Consider they are the same under 0.2m
    {
        // While we didn't iterate through all way
        // Compute each new point
        
        // 2 cases : either keep turning, either we are in the good direction
        // so go straight
        if(abs(getDirection(xPrime, x2) - xPrime.th) < (M_PI / 50))
        {
            // We are in the ~appropriate direction (2% of pi of error), so
            // go straight. We cannot go exactly in the good direction since 
            // delta t is not infinitely small
            xPrime.x = xc.x + d * sin(xPrime.th);
            xPrime.y = xc.y + d * cos(xPrime.th);
        }
        else
        {
            // Keep turning
            xc.x = xPrime.x - R * sin(xPrime.th);
            xc.y = xPrime.y + R * cos(xPrime.th); 
            
            xPrime.th = xPrime.th + beta;
            xPrime.x = xc.x + R * sin(xPrime.th);
            xPrime.y = xc.y - R * cos(xPrime.th);
        }
        
        // After computing a new point, we can check wheter it's in an area
        // of the matrix already counter or not
        
        // Compute the value of the new point in the matrix plan
        iPrime = ((int)floor(xPrime.x / res)) + map_origin_x;
        jPrime = ((int)floor(xPrime.y / res)) + map_origin_y;
        
        // And only if it's different, push it back to the structure
        if(iPrime != i.back() || jPrime != j.back())
        {
            i.push_back(iPrime);
            j.push_back(jPrime);
        }
    }
    
    // Now, we have the full path to point 1 to point 2. Just check if there
    // is some obstacle in the middle or not
    std::vector<int>::iterator itI = i.begin();
    std::vector<int>::iterator itJ = j.begin();
    for(itI = i.begin(); itI != i.end(); itI++)
    {
        if(map.at<uchar>(*itI, *itJ) == 255)
        {
            return true;
        }
        itJ++;
    }
    
    return false;
}

std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    
    // First, find the nearest node to the goal
    float bestDistance = distance(ptrTable[0]->location, x_goal);
    int bestI = 0;
    for(int i = 0; i < count; i++)
    {
        if(distance(ptrTable[i]->location, x_goal) < bestDistance)
        {
            bestDistance = distance(ptrTable[i]->location, x_goal);
            bestI = i;
        }
    }
    
    // Then, track parents one by one and add them to the traj
    std::vector<traj> path;
    node currentNode = ptrTable[bestI];
    while(currentNode.idx_parent != NULL)
    {
        // Create equivalent 'traj' of the current node
        traj *trajPoint = new traj();
        trajPoint->x = currentNode.location.x;
        trajPoint->y = currentNode.location.y;
        trajPoint->th = currentNode.location.th;
        trajPoint->d = currentNode.d;
        trajPoint->alpha = currentNode.alpha;
        
        // Add it to our vector
        path.push_back(*trajPoint);
        
        trajPoint = NULL;
        
        // Go to the parent
        currentNode = ptrTable[currentNode.idx_parent];
    }
    
    // DEBUG
    visualizeTree(path);
    
    return path;
}