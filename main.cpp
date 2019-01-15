#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>


using namespace cv;
using namespace std;

#include<iostream>
#include<vector>
#include<queue>
#include<string.h>
#include<limits.h>

using namespace std;

vector<int> x1;
 
// To add an edge
void addEdge(vector <pair<int, int> > adj[], int u,
                                     int v, int wt)
{
    adj[u].push_back(make_pair(v, wt));
    //adj[v].push_back(make_pair(u, wt));
}

bool bfs(vector <pair<int, int> > adj[], int V, int source, int dest, int parent[]){
    //cout<<"bfs called"<<endl;
    bool visited[V];
    memset(visited, false, V*sizeof(bool));
    memset(parent, -1, V*sizeof(int));

    queue<int> Q;
    Q.push(source);
    visited[source] = true;
    //parent[source] = -1;
    //cout<<"going inside"<<endl;
    while(!Q.empty()){

        int u = Q.front();
        Q.pop();

        for (std::vector<std::pair<int, int> >::iterator it = adj[u].begin(); it!=adj[u].end(); it++)
        {
           // cout<<"more in"<<endl;
            int v = it->first;
            int w = it->second;
            if(visited[v] == false && w > 0){
                Q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
        //cout<<"out_______________________________"<<endl; 
    }
    //cout<<"+++++++++++++++++++++++!!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@"<<endl;

    return (visited[dest] == true); 
}

void bfsNew(vector <pair<int, int> > adj[], int V, int source, bool visited[]){
    //bool visited[V];
    memset(visited, false, sizeof(visited));
    //memset(parent, -1, sizeof(parent));

    queue<int> Q;
    Q.push(source);
    visited[source] = true;
    //parent[source] = -1;

    while(!Q.empty()){

        int u = Q.front();
        Q.pop();

        for (std::vector<std::pair<int, int> >::iterator it = adj[u].begin(); it!=adj[u].end(); it++)
        {
            int v = it->first;
            int w = it->second;
            if(visited[v] == false && w > 0){
                Q.push(v);
                //parent[v] = u;
                visited[v] = true;
            }
        }
    }
}


// void dfs(vector <pair<int, int> > adj[], int source, bool visited[]){
//     //cout<<"inside function"<<endl;
//     if(source==167928){
//         cout<<"inside"<<endl;
//         //167929 //260
//     }
//     visited[source] = true;
//     for (std::vector<std::pair<int, int> >::iterator it = adj[source].begin(); it!=adj[source].end(); it++)
//     {
//         if(source == 167928){
//             cout<<"inside neighbours"<<endl;
//         }
//         int i = it->first;
//         int j = it->second;
//         if(!visited[i] && j > 0){
//             //cout<<"fn called with" << source << " and " << i <<" weight " << j <<endl;
//             dfs(adj, i, visited);
//         }
//    }
// }

int fordFulkerson(vector <pair<int, int> > residualGraph[], int V, int source, int dest, bool visited[]){
//cout<<"inside FF"<<endl;
    //vector<pair<int, int> > residualGraph[V];
    // for (int u = 0; u < V; u++)
    // {
    //     for (std::vector<std::pair<int, int> >::iterator it = adj[u].begin(); it!=adj[u].end(); it++)
    //     {
    //         int v = it->first;
    //         int w = it->second;
    //         //addEdge(residualGraph, u, v, w);
    //     }
    // }
    
    int parent[V];
    //memset(visited, -1, V*sizeof(int));

    int max_flow = 0;
    int count = 0;
    while(bfs(residualGraph, V, source, dest, parent)){
       //  for(int q =0; q<V; q++){
       //      //if(parent[q] != 0)
       //      cout<<q<<" :: "<<parent[q]<<endl;
       //  }
       // break;
        //cout<<"loop started again"<<endl;
        cout<<"New path "<<count<<" found!"<<endl;
        int path_flow = INT_MAX;
        //int p = 0;
        for (int v=dest; v!=source; v=parent[v])
        {
            //cout<<p<<endl;p++;
            int u = parent[v];
            int weight;
            for (std::vector<std::pair<int, int> >::iterator it = residualGraph[u].begin(); it!=residualGraph[u].end(); it++)
            {
                int i = it->first;
                int j = it->second;
                // if(j!=258){
                //     cout<<j<<endl;
                // }
                if(v==i){
                    weight=j;
                }
            }
            //if(path_flow==weight){
               // cout<<"again equal"<<endl;
           // }
            //cout<<"weight: "<<weight<<"   path_flow: "<<path_flow<<endl;
            path_flow = min(path_flow, weight);
        }
        
        //cout<< path_flow<<endl;

        // break;

        for (int v=dest; v!=source; v=parent[v])
        {
            //cout<<p<<endl;p++;
            int u = parent[v];
            int weight;
            for (std::vector<std::pair<int, int> >::iterator it = residualGraph[u].begin(); it!=residualGraph[u].end(); it++)
            {
                int i = it->first;
                int j = it->second;
                if(v==i){
                    if(j==path_flow){
                       // cout<<"weight: "<<j<<"   path_flow: "<<path_flow<<endl;
                        x1.push_back(i);
                        x1.push_back(u);
                        //cout<<"cut: "<<i<<endl;
                        //drawPixel(i);
                        //break;
                    }
                }
            }
        }

        //cout<<"found cut"<<endl;
        //break;


        for (int v=dest; v!=source; v=parent[v])
        {
            int u = parent[v];
            int weight;
            for (std::vector<std::pair<int, int> >::iterator it = residualGraph[u].begin(); it<residualGraph[u].end(); it++)
            {

                int i = it->first;
                int j = it->second;
                if(j==path_flow){
                   // cout<<"before "<<it->second<<endl;
                   it->second = it->second - path_flow;
                    //cout<<"after "<<it->second<<endl;
                }
            }

    //         //delete the edge if the weight becomes 0, but its already handled in BFS
    //                 // if(it->second == 0){
    //                 //     residualGraph[u].erase(it);
    //                 //     cout<<"edge deleted"<<endl;
    //                 // }
            //break;
            bool found = false;
            for (std::vector<std::pair<int, int> >::iterator it = residualGraph[v].begin(); it!=residualGraph[v].end(); it++)
            {
                int i = it->first;
                //int w = it->second;
                if(i==u){
                    it->second = it->second + path_flow;
                    found = true;
                }
            }
            if(!found){
                residualGraph[v].push_back(make_pair(u, path_flow));
            }
        }
        //break;
        cout<<"Path "<<count<<" removed."<<endl;
        count++;
        max_flow += path_flow; 

   }

    // //find minimum cut
    cout<<"finding minimum graph cut"<<endl;
    bfsNew(residualGraph, V, source, visited);

    return max_flow;
}
 

//main
int main( int argc, char** argv )
{
    if(argc!=4){
        cout<<"Usage: ../seg input_image initialization_file output_mask"<<endl;
        return -1;
    }
    
    // Load the input image
    // the image should be a 3 channel image by default but we will double check that in teh seam_carving
    Mat in_image;
    in_image = imread(argv[1]/*, CV_LOAD_IMAGE_COLOR*/);
   
    if(!in_image.data)
    {
        cout<<"Could not load input image!!!"<<endl;
        return -1;
    }

    if(in_image.channels()!=3){
        cout<<"Image does not have 3 channels!!! "<<in_image.depth()<<endl;
        return -1;
    }
    
    // the output image
    Mat out_image = in_image.clone();
    
    ifstream f(argv[2]);
    if(!f){
        cout<<"Could not load initial mask file!!!"<<endl;
        return -1;
    }
    
    int width = in_image.cols;
    int height = in_image.rows;
    
    double minVal, maxVal;
    Mat gray;
    cvtColor(in_image, gray, CV_BGR2GRAY);
    // minMaxLoc(gray, &minVal, &maxVal);

    //applying sobel filter
    Mat sobelgrey;
    Mat dx, dy;
    //calculating X-Gradient
    Sobel(gray, dx, CV_64F, 1, 0);
    
    //calculating Y-Gradient
    Sobel(gray, dy, CV_64F, 0, 1);
    
    //Total Gradient
    magnitude(dx, dy, sobelgrey);

    //Normalizing the matrix using its maximum value (from github:: loc-trinh/seamCarving) 
    
    minMaxLoc(sobelgrey, &minVal, &maxVal);
    sobelgrey = sobelgrey/maxVal*255;

    //converting the image back to unsigned character
    sobelgrey.convertTo(sobelgrey, CV_32S);
    minMaxLoc(sobelgrey, &minVal, &maxVal);
    
    //cout<<maxVal<<endl;

    vector<int> weights(width*height);

    for(int y = 0; y<height; y++){
        for(int x = 0; x<width; x++){
            weights[y*width+x] = sobelgrey.at<int>(y, x);
            //if(weights[y*width+x] != 0){
              //  cout<<weights[y*width+x]<<endl;
            //}
        }
    }
    //maxVal = 260;
    int V = width*height;
    vector<pair<int, int> > adj[V+2];

    int index, avg1, avg2, avg3, avg4, n1, n2, n3, n4;
    for(int y = 0; y<height; y++){
        for(int x = 0; x<width; x++){
            index = y*width+x;
            if(y==0){
                if(x==0){
                    n1 = index+1;
                    avg1 = (abs(weights[index]) + abs(weights[n1]))/2;
                    avg1 = abs(maxVal - avg1);
                    addEdge(adj, index, n1, avg1);
                    n2 = index+width;
                    avg2 = (abs(weights[index]) + abs(weights[n2]))/2;
                    avg2 = abs(maxVal - avg2);
                    addEdge(adj, index, n2, avg2);
                }
                else if(x==width-1){
                    n2 = index+width;
                    avg2 = (abs(weights[index]) + abs(weights[n2]))/2;
                    avg2 = abs(maxVal - avg2);
                    addEdge(adj, index, n2, avg2);
                    n3 = index-1;
                    avg3 = (abs(weights[index]) + abs(weights[n3]))/2;
                    avg3 = abs(maxVal - avg3);
                    addEdge(adj, index, n3, avg3);
                }
                else{
                    n1 = index+1;
                    avg1 = (abs(weights[index]) + abs(weights[n1]))/2;
                    avg1 = abs(maxVal - avg1);
                    addEdge(adj, index, n1, avg1);
                    n2 = index+width;
                    avg2 = (abs(weights[index]) + abs(weights[n2]))/2;
                    avg2 = abs(maxVal - avg2);
                    addEdge(adj, index, n2, avg2);
                    n3 = index-1;
                    avg3 = (abs(weights[index]) + abs(weights[n3]))/2;
                    avg3 = abs(maxVal - avg3);
                    addEdge(adj, index, n3, avg3);
                }
            }
            else if(y==height-1){
                if(x==0){
                    n4 = index-width;
                    avg4 = (abs(weights[index]) + abs(weights[n4]))/2;
                    avg4 = abs(maxVal - avg4);
                    addEdge(adj, index, n4, avg4);
                    n1 = index+1;
                    avg1 = (abs(weights[index]) + abs(weights[n1]))/2;
                    avg1 = abs(maxVal - avg1);
                    addEdge(adj, index, n1, avg1);
                }
                else if(x==width-1){
                    n3 = index-1;
                    avg3 = (abs(weights[index]) + abs(weights[n3]))/2;
                    avg3 = abs(maxVal - avg3);
                    addEdge(adj, index, n3, avg3);
                    n4 = index-width;
                    avg4 = (abs(weights[index]) + abs(weights[n4]))/2;
                    avg4 = abs(maxVal - avg4);
                    addEdge(adj, index, n4, avg4);
                }
                else{
                    n3 = index-1;
                    avg3 = (abs(weights[index]) + abs(weights[n3]))/2;
                    avg3 = abs(maxVal - avg3);
                    addEdge(adj, index, n3, avg3);
                    n4 = index-width;
                    avg4 = (abs(weights[index]) + abs(weights[n4]))/2;
                    avg4 = abs(maxVal - avg4);
                    addEdge(adj, index, n4, avg4);
                    n1 = index+1;
                    avg1 = (abs(weights[index]) + abs(weights[n1]))/2;
                    avg1 = abs(maxVal - avg1);
                    addEdge(adj, index, n1, avg1);
                }
            }
            else{
                if(x==0){
                    n4 = index-width;
                    avg4 = (abs(weights[index]) + abs(weights[n4]))/2;
                    avg4 = abs(maxVal - avg4);
                    addEdge(adj, index, n4, avg4);
                    n1 = index+1;
                    avg1 = (abs(weights[index]) + abs(weights[n1]))/2;
                    avg1 = abs(maxVal - avg1);
                    addEdge(adj, index, n1, avg1);
                    n2 = index+width;
                    avg2 = (abs(weights[index]) + abs(weights[n2]))/2;
                    avg2 = abs(maxVal - avg2);
                    addEdge(adj, index, n2, avg2);
                }
                else if(x==width-1){
                    n2 = index+width;
                    avg2 = (abs(weights[index]) + abs(weights[n2]))/2;
                    avg2 = abs(maxVal - avg2);
                    addEdge(adj, index, n2, avg2);
                    n3 = index-1;
                    avg3 = (abs(weights[index]) + abs(weights[n3]))/2;
                    avg3 = abs(maxVal - avg3);
                    addEdge(adj, index, n3, avg3);
                    n4 = index-width;
                    avg4 = (abs(weights[index]) + abs(weights[n4]))/2;
                    avg4 = abs(maxVal - avg4);
                    addEdge(adj, index, n4, avg4);
                }
                else{
                    n1 = index+1;
                    avg1 = (abs(weights[index]) + abs(weights[n1]))/2;
                    avg1 = abs(maxVal - avg1);
                    // if(avg1 != 260){
                    //     x1.push_back(n1);
                    // }
                    addEdge(adj, index, n1, avg1);
                    n2 = index+width;
                    avg2 = (abs(weights[index]) + abs(weights[n2]))/2;
                    avg2 = abs(maxVal - avg2);
                    addEdge(adj, index, n2, avg2);
                    n3 = index-1;
                    avg3 = (abs(weights[index]) + abs(weights[n3]))/2;
                    avg3 = abs(maxVal - avg3);
                    addEdge(adj, index, n3, avg3);
                    n4 = index-width;
                    avg4 = (abs(weights[index]) + abs(weights[n4]))/2;
                    avg4 = abs(maxVal - avg4);
                    addEdge(adj, index, n4, avg4);
                }
            }

        }
    }
    //cout<<maxVal;
    //printGraph(adj, V);
    
    // for (int u = 0; u < V; u++)
    // {
    //     for (std::vector<std::pair<int, int> >::iterator it = adj[u].begin(); it!=adj[u].end(); it++)
    //     {
    //         int v = it->first;
    //         int w = it->second;
    //         if(w==0){
    //             cout<<"found 0 weight"<<endl;
    //         }
    //         else{
    //             cout<<"not"<<endl;
    //         }
    //     }
    // }
    //printGraph(adj, width*height);

    //visitedFromSource && maybe visitedFromDestination also required
    //bool visited[V];
    //memset(visited, false, V*sizeof(bool));
    //dfs(adj, source, visited);

    //cout<<"Max flow -> "<<fordFulkerson(adj, V, 0, 5, visited)<<endl;
    // cout<<"vertices belonging to source are:"<<endl;
    // for(int i=0; i<V; i++){
    //     cout<< i << " == "<< visited[i]<<endl;
    // }

    int n;
    f>>n;
    //int source = 250*width+250;
    //int sink = 0*width + 50;
    int source = V;
    int sink = V+1;

    //get the initil pixels
    for(int i=0;i<n;++i){
        int x, y, t;
        f>>x>>y>>t;
        
       if(x<0 || x>=width || y<0 || y>=height){
           cout<<"I valid pixel mask!"<<endl;
           return -1;
        }

        Vec3b pixel;
        pixel[0] = 0;
        pixel[1] = 0;
        pixel[2] = 0;
        
        if(t==1){
            pixel[2] = 255;
            addEdge(adj, source, ((y-1)*width)+(x-1), INT_MAX);
        } else {
            pixel[0] = 255;
            addEdge(adj, ((y-1)*width)+(x-1), sink, INT_MAX);
        }
        out_image.at<Vec3b>(y, x) = pixel;
    
    }

    bool visited[V];
    memset(visited, false, V*sizeof(bool));
    // //dfs(adj, source, visited);
    //cout<<"value size ::: " << V<<endl;
    cout<<"Max flow -> "<<fordFulkerson(adj, V+2, source, sink, visited)<<endl;
    //cout<<"outside FF"<<endl;

    // //cout<<"vertices belonging to source are:"<<endl;

    //cout<<"size:::"<<x1.size()<<endl;
    Vec3b pixel;
    pixel[0] = 0;
    pixel[1] = 0;
    pixel[2] = 255;
    for(int q=0;q<x1.size(); q++){
        int x, y;
        x = ((int)x1[q]/640)+1;
        y = ((int)x1[q]%640)+1;
        out_image.at<Vec3b>(x, y) = pixel;
    }
        cout<<"coloring foreground and backgroud pixels"<<endl;
        for(int y = 0; y<height; y++){
            for(int x = 0; x<width; x++){
                index = y*width+x;
                if(visited[index]){
                    pixel[0] = 0;
                    pixel[1] = 0;
                    pixel[2] = 255;
                    out_image.at<Vec3b>(y, x) = pixel;
                }
                else{
                    pixel[0] = 255;
                    pixel[1] = 0;
                    pixel[2] = 0;
                    out_image.at<Vec3b>(y, x) = pixel;
                }
            }
        }
        
    
    // write it on disk

    imwrite( argv[3], out_image);
    
    // also display them both
    
    namedWindow( "Original image", WINDOW_AUTOSIZE );
    namedWindow( "Show Marked Pixels", WINDOW_AUTOSIZE );
    imshow( "Original image", in_image );
    imshow( "Show Marked Pixels", out_image );
    waitKey(0);
    return 0;
}

