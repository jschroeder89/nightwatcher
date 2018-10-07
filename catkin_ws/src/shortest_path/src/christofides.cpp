#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <iterator>
#include "std_msgs/UInt8MultiArray.h"
#include "yaml-cpp/yaml.h"
using namespace std;

#include <sys/time.h>
ros::Publisher pub;
std_msgs::UInt8MultiArray beacon_order;
// Boruvka's algorithm to find Minimum Spanning
// Tree of a given connected, undirected and
// weighted graph
#include <stdio.h>

// a structure to represent a weighted edge in graph
struct Edge
{
    int src, dest;
    float_t weight;
    bool visited;
};
 
// a structure to represent a connected, undirected
// and weighted graph as a collection of edges.
struct Graph
{
    // V-> Number of vertices, E-> Number of edges
    int V, E;
 
    // graph is represented as an array of edges.
    // Since the graph is undirected, the edge
    // from src to dest is also edge from dest
    // to src. Both are counted as 1 edge here.
    Edge* edge;
};
 
// A structure to represent a subset for union-find
struct subset
{
    int parent;
    int rank;
};
 
// Function prototypes for union-find (These functions are defined
// after boruvkaMST() )
int find(struct subset subsets[], int i);
void Union(struct subset subsets[], int x, int y);
struct Graph* createGraph(int V, int E);
void hierholzer(struct Graph* graph, int *eulertour);
 
// The main function for MST using Boruvka's algorithm
struct Graph* boruvkaMST(struct Graph* graph)
{
	int count = 0;
    // Get data of given graph
    int V = graph->V, E = graph->E;
    Edge *edge = graph->edge;
 
	struct Graph* outGraph = createGraph(V, V-1);
    // Allocate memory for creating V subsets.
    struct subset *subsets = new subset[V];
 
    // An array to store index of the cheapest edge of
    // subset.  The stored index for indexing array 'edge[]'
    int *cheapest = new int[V];
 
    // Create V subsets with single elements
    for (int v = 0; v < V; ++v)
    {
        subsets[v].parent = v;
        subsets[v].rank = 0;
        cheapest[v] = -1;
    }
 
    // Initially there are V different trees.
    // Finally there will be one tree that will be MST
    int numTrees = V;
    float_t MSTweight = 0;
 
    // Keep combining components (or sets) until all
    // compnentes are not combined into single MST.
    while (numTrees > 1)
    {
         // Everytime initialize cheapest array
          for (int v = 0; v < V; ++v)
           {
               cheapest[v] = -1;
           }
 
        // Traverse through all edges and update
        // cheapest of every component
        for (int i=0; i<E; i++)
        {
            // Find components (or sets) of two corners
            // of current edge
            int set1 = find(subsets, edge[i].src);
            int set2 = find(subsets, edge[i].dest);
 
            // If two corners of current edge belong to
            // same set, ignore current edge
            if (set1 == set2)
                continue;
 
            // Else check if current edge is closer to previous
            // cheapest edges of set1 and set2
            else
            {
               if (cheapest[set1] == -1 ||
                   edge[cheapest[set1]].weight > edge[i].weight)
                 cheapest[set1] = i;
 
               if (cheapest[set2] == -1 ||
                   edge[cheapest[set2]].weight > edge[i].weight)
                 cheapest[set2] = i;
            }
        }
 
        // Consider the above picked cheapest edges and add them
        // to MST
        for (int i=0; i<V; i++)
        {
            // Check if cheapest for current set exists
            if (cheapest[i] != -1)
            {
                int set1 = find(subsets, edge[cheapest[i]].src);
                int set2 = find(subsets, edge[cheapest[i]].dest);
 
                if (set1 == set2)
                    continue;
                MSTweight += edge[cheapest[i]].weight;
                printf("Edge %d-%d included in MST\n",
                       edge[cheapest[i]].src, edge[cheapest[i]].dest);
                outGraph->edge[count] = edge[cheapest[i]];
                count++;
 
                // Do a union of set1 and set2 and decrease number
                // of trees
                Union(subsets, set1, set2);
                numTrees--;
            }
        }
    }
 
    printf("Weight of MST is %f\n", MSTweight);
    return outGraph;
}
 
 int findUnevenNumber(struct Graph* graph){
	 int grade[graph->V];
	 int count = 0;
	 for(int i = 0; i < graph->V; i++){
		grade[i] = 0;
	 }
	 for(int i = 0; i < graph->E; i++){
		 grade[graph->edge[i].dest]++;
		 grade[graph->edge[i].src]++;
	 }
	 for(int i = 0; i < graph->V; i++){
		cout << i << ": " << grade[i] << endl;
		if(grade[i] % 2 == 1)
		{
			count++;
			cout << "Number of uneven-graded vertices so far: " << count << endl;
		}
	 }
	 return count;
 }
 
 int *findUneven(struct Graph* graph, int num, int *uneven){
	 int grade[graph->V];
	 int count;
	 for(int i = 0; i < graph->V; i++){
		grade[i] = 0;
	 }
	 for(int i = 0; i < graph->E; i++){
		 grade[graph->edge[i].dest]++;
		 grade[graph->edge[i].src]++;
	 }
	cout << "TEST" << endl;
	 count = 0;
	 for(int i = 0; i < graph->V; i++){
		if(grade[i] % 2 == 1)
		{
			cout << "Test No. " << i << endl;
			uneven[count] = i;
			count++;
			
		}
	 }
	 
	 
	 
	 
 }
// Creates a graph with V vertices and E edges
struct Graph* createGraph(int V, int E)
{
    Graph* graph = new Graph;
    graph->V = V;
    graph->E = E;
    graph->edge = new Edge[E];
    return graph;
}
 
// A utility function to find set of an element i
// (uses path compression technique)
int find(struct subset subsets[], int i)
{
    // find root and make root as parent of i
    // (path compression)
    if (subsets[i].parent != i)
      subsets[i].parent =
             find(subsets, subsets[i].parent);
 
    return subsets[i].parent;
}
 
// A function that does union of two sets of x and y
// (uses union by rank)
void Union(struct subset subsets[], int x, int y)
{
    int xroot = find(subsets, x);
    int yroot = find(subsets, y);
 
    // Attach smaller rank tree under root of high
    // rank tree (Union by Rank)
    if (subsets[xroot].rank < subsets[yroot].rank)
        subsets[xroot].parent = yroot;
    else if (subsets[xroot].rank > subsets[yroot].rank)
        subsets[yroot].parent = xroot;
 
    // If ranks are same, then make one as root and
    // increment its rank by one
    else
    {
        subsets[yroot].parent = xroot;
        subsets[xroot].rank++;
    }
}
 
 
 struct Graph* matching(struct Graph* graph){
	 
 }
 
 bool isIn(int in[], int num, int test){
	bool out = false;
	for(int i = 0; i < num; i++){
		if(in[i] == test){
			out = true;
		}
	}
	return out;
}

void del(int *eulertour, int pos, int max){
	for(int i = pos; i < max; i++){
		eulertour[i] = eulertour[i+1];
	}
}

void swap(int *tour, int pos1, int pos2){
	int size = pos2-pos1+1;
	int help [size];
	for(int i = 0; i < size; i++){
		help[i] = tour[pos2-i];
	}
	for(int i = 0; i < size; i++){
		tour[pos1 + i] = help[i];
	}
}

 
// Driver program to test above functions
int detMST(float_t* inTree, int bc)
{
	//Array in Matrix umwandeln
	float_t inTree2d [bc+1][bc+1];
	for(int i = 0; i < bc + 1; i++){
		for(int j = 0; j < bc + 1; j++){ 
			inTree2d[i][j] = inTree[i*(bc+1) + j];
		}
	}
	
	cout << "2D-Matrix created." << endl;
    int V = bc+1;  // Number of vertices in graph
    int E = ((bc+1)*bc)/2;  // Number of edges in graph
    struct Graph* graph = createGraph(V, E);
	
	cout << "Graph created" << endl;
	int count = 0;
	for(int i = 0; i < bc + 1; i++){
		for(int j = 0; j < bc + 1; j++){ 
			if(i < j){
			cout << "Create edge " << count << endl;
			graph->edge[count].src = i;
			graph->edge[count].dest = j;
			graph->edge[count].weight = inTree2d[i][j];
			graph->edge[count].visited = false;
			cout << "Weight of the edge from " << i << " to " << j << " is: " << inTree2d[i][j] << endl;
			count++;
			}
		}
	}
	//Minimum Spanning Tree
	cout << "Edges ready" << endl;
    graph = boruvkaMST(graph);
    for(int i = 0; i < graph->E; i++){
		cout << "Weight of edge " << i << ": " << graph->edge[i].weight << endl;
	}
	//Find vertices with uneven grade
	int num = findUnevenNumber(graph);
	cout << "Number of uneven-graded vertices found: " << num << endl;
	int uneven [num];
	cout << "Array with " << num << " Elements initialized." << endl;
	findUneven(graph, num, uneven);
	for(int i = 0; i < num; i++){
		cout << uneven[i] << endl;
	}
	
	//Do a nearest-neighbour-matching on those vertices
	int matched [num];
	for(int i = 0; i < num; i++){matched[i] = 100000;}
	int matchedCount = 0;
	for(int i = 0; i < num - 1; i++){
		float_t help = 100000;
		if(!isIn(matched, num, uneven[i])){
			for(int j = i+1; j < num; j++){
				if(!isIn(matched, num, uneven[j]) && inTree2d[i][j] < help){
					help = inTree2d[i][j];
					cout << "Variable help: " << help << endl;
					matched[matchedCount] = uneven[i];
					matched[matchedCount+1] = uneven[j];
				}		
			}
			matchedCount = matchedCount + 2;
		}
	}
	cout << num << endl;
	cout << "Matching: ";
	for(int i = 0; i < num; i++){
		cout << "(" << i << ") ";
		cout << matched[i] << " ";
	}
	cout << endl;

	//Add matching edges to spanning tree
	int ex = graph->E; //number of existing edges
	for(int i = 0; i < num/2; i++){
			graph->edge[ex+i].src = matched[2*i];
			graph->edge[ex+i].dest = matched[2*i+1];
			graph->edge[ex+i].weight = inTree2d[matched[2*i]][matched[2*i+1]];
			graph->edge[ex+i].visited = false;
			graph->E = ex+i+1;
	}
	cout << "Adding matching edges complete." << endl;

	int eulertour[graph->E];
	
	for(int i = 0; i < graph->E; i++){
		eulertour[i] = 0;
	}
	//perform Hierholzer's algorithm to find a tour that includes all edges.
	hierholzer(graph, eulertour);
	
	float_t tourLength = 0;
	for(int i = 0; i < graph->E; i++){
		tourLength += graph->edge[i].weight;
	}
	cout << "Maximum length of Tour: " << tourLength << endl;
	cout << "Tour: 0";
	for(int i = 1; i < graph->E+1; i++){
		cout << "-" << eulertour[i];
	}
	cout << endl;
	
	//Delete duplicate vertices to shorten the tour
	/*
	 * Take the tour A-B-C as an example. B would be the duplicate vertex. 
		A---B
		 \  |
		  \ |
		   \|
		    C
		Deleting B shortens the tour.
	
	*/
	int finalTour [graph->V+1];
	int duplicates [graph->V];
	for(int i = 0; i < graph->V; i++){
		duplicates[i] = 0;
	}
	
	for(int i = 0; i < graph->E-1; i++){
		if(duplicates[eulertour[i]] == 0){
			duplicates[eulertour[i]] = 1;
		}else{
			cout << "Delete " << eulertour[i] << endl;
			del(eulertour, i, graph->E);
		}	
	}
	for(int i = 0; i < graph->V+1; i++){
		finalTour[i] = eulertour[i];
	}
	//Calculate length of tour
	tourLength = 0;
	for(int i = 1; i < graph->V+1; i++){
		float_t helpme = inTree2d[finalTour[i-1]][finalTour[i]];
		cout << "Distance from " << finalTour[i-1] << " to " << finalTour[i] << ": " << helpme << endl;
		tourLength += inTree2d[finalTour[i-1]][finalTour[i]];
	}
	cout << "Actual length of tour: " << tourLength << endl;
	
	cout << "Final tour: 0";
	for(int i = 1; i < graph->V+1; i++){
		cout << "-" << finalTour[i];
	}
	cout << endl;
	
	//Swap edges that cross each other
	//2-opt-heuristic
	int betterCount = 0;
	goto start_2_opt;
	start_2_opt:
	for(int i = 1; i < graph->V-1; i++){
		for(int j = i+2; j < graph->V-1; j++){
			//cout << "Calculating distance between " << finalTour[i] << "/" << finalTour[i+1] << " and " << finalTour[j] << "/" << finalTour[j+1] << endl;
			float_t edgeDist = inTree2d[finalTour[i]][finalTour[i+1]]+inTree2d[finalTour[j]][finalTour[j+1]];
			float_t crossDist = inTree2d[finalTour[i]][finalTour[j]]+inTree2d[finalTour[i+1]][finalTour[j+1]];
			if(crossDist < edgeDist){
				swap(finalTour, i+1, j);
				betterCount++;
				cout << "Final tour: 0";
				for(int i = 1; i < graph->V+1; i++){
					cout << "-" << finalTour[i];
				}
				cout << endl;
				cout << "Verbesserungsdurchgang " << betterCount << endl;
				goto start_2_opt;
			}
		}
	}
	
	tourLength = 0;
	for(int i = 1; i < graph->V+1; i++){
		float_t helpme = inTree2d[finalTour[i-1]][finalTour[i]];
		cout << "Distance from " << finalTour[i-1] << " to " << finalTour[i] << ": " << helpme << endl;
		tourLength += inTree2d[finalTour[i-1]][finalTour[i]];
	}
	cout << "Length of tour after improvement: " << tourLength << endl;
	beacon_order.data.push_back(0);
	cout << "Final tour: 0";
	for(int i = 1; i < graph->V+1; i++){
		cout << "-" << finalTour[i];
		beacon_order.data.push_back(finalTour[i]);	
	}
	beacon_order.data.push_back(0);
	YAML::Node shortest_path;
	YAML::Emitter out;
	finalTour[0] = 0;
	//out << "shortest_path";
	out << YAML::Flow;
	out << YAML::BeginSeq << finalTour[0] << finalTour[1] << finalTour[2] << finalTour[3] << finalTour[4] << finalTour[5] << finalTour[6] << finalTour[7] << YAML::EndSeq;
	ofstream fout("shortest_path.yaml");
	fout << out.c_str();
	cout << endl;
	
    return 0;
}

bool isUnvisited(Edge* edge, int in){
	if((edge->src == in || edge->dest == in) && edge->visited == false){
		return true;
	}else{
		return false;
	}
}

struct Node{
	int number;
	Edge* edges;
	int edgeCount;
};

void insertInTour(int *tour, int num, int pos, int size){
	for(int i = size; i > pos; i--){
		//Print out the tour in every iteration
		cout << "Tour: " << tour[0];
		for(int j = 1; j < size+1; j++){
			cout << "-" << tour[j];
		}
		cout << endl;
		
		tour[i] = tour[i-1];
		cout << "Inserting " << tour[i-1] << " in tour at pos " << i << endl;
	}
	tour[pos] = num;
	
}

void hierholzer(struct Graph* graph, int *eulertour){
	int visitedCount = 0;
	int next; 
	Node nodes [graph->V];
	Node help;
	for(int i = 0; i < graph->V; i++){
		help.number = i;
		help.edgeCount = 0;
		for(int j = 0; j < graph->E; j++){
			if(graph->edge[j].src == i || graph->edge[j].dest == i){
				help.edgeCount++;
			}
		}
		help.edges = new Edge [help.edgeCount];
		help.edgeCount = 0;
		for(int j = 0; j < graph->E; j++){
			if(graph->edge[j].src == i || graph->edge[j].dest == i){
				help.edges[help.edgeCount] = graph->edge[j];
				help.edgeCount++;
				cout << "For node " << i << ": Adding edge " << j << endl;
			}
		}
		nodes[i] = help;
	}
	Node* active = &nodes[0];
	Node* nextNode;
	Node* subtourStart;
	cout << "Active node: " << active->number << endl;
	bool subtourMissing = false;
	bool subtourActive = false;
	eulertour[0] = 0;
	list<int> subtour;
	Edge* helpEdge;
	Edge* helpEdge2;
	while(visitedCount < graph->E){
		//while
		if(subtourMissing){
			for(int i = 0; i < visitedCount; i++){
				nextNode = &nodes[eulertour[i] ];
				for(int j = 0; j < nextNode->edgeCount; j++){
					helpEdge = &nextNode->edges[j];
					if(isUnvisited(helpEdge, nextNode->number)){
						active = nextNode;
						subtourStart = active;
						subtourActive = true;
						cout << "😎 Active node is now: " << active->number << endl;
						visitedCount--;
						break;
					}
				}
			}
		}	
		for(int i = 0; i != active->edgeCount; i++){
			helpEdge = &active->edges[i];
			cout << "Edge from " << helpEdge->src << " to " << helpEdge->dest << " is visited: " << helpEdge->visited << endl;
			if(isUnvisited(helpEdge, active->number)){
				if(helpEdge->src == active->number){
					next = helpEdge->dest;
				}else{
					next = helpEdge->src;
				}
				cout << "||||Next node is: " << next << endl;
				nextNode = &nodes[next];
				for(int k = 0; k != nextNode->edgeCount; k++){
					helpEdge2 = &nextNode->edges[k];
					if(isUnvisited(helpEdge2, active->number)){
						helpEdge2->visited = true;
						cout << "Edge from " << helpEdge2->src << " to " << helpEdge2->dest << " is set to visited: " << helpEdge2->visited << endl;
						break;
					}
				}
				helpEdge->visited = true;
				cout << "Edge from " << helpEdge->src << " to " << helpEdge->dest << " is set to visited: " << helpEdge->visited << endl;
				visitedCount++;
				if(subtourActive){
					if(nextNode == subtourStart){
						int position = 0;
						for(int j = 0; j < visitedCount; j++){
							if(eulertour[j] == subtourStart->number){
								position = j;
							}
						}
						cout << "Subtour size is: " << subtour.size() << endl;
						while(!subtour.empty()){
							insertInTour(eulertour, subtour.front(), position, visitedCount);
							cout << "Put " << subtour.front() << " in eulertour at place " << position << endl;
							cout << "Had to shuffle " << visitedCount - position << " elements." << endl;
							subtour.pop_front();
							position++;
						}
						subtourActive = false;
					}else{
						subtour.push_front(next);
						visitedCount++;
					}
				}else{
					eulertour[visitedCount] = next;
					cout << "Put " << next << " in eulertour at place " << visitedCount << endl;
					cout << "Tour: " << eulertour[0];
					for(int j = 1; j < visitedCount; j++){
						cout << "-" << eulertour[j];
					}
					cout << endl;
				}
				break;
			} 	
		}
		if(&nodes[next] == active){
			
			cout << endl << "⮚⮚⮚ At least one subtour is still missing. ⮘⮘⮘" << endl << endl;
			subtourMissing = true;
			
		}else{
			subtourMissing = false;
		}
		active = &nodes[next];
		cout << endl << "Active node is now: " << active->number << endl;
		
	}
	

}

// Thanks to Anukul Chand for modifying above code.




int isNumber(char in){
		if(in == '1'  || in == '2' || in == '3' || in == '4' || in == '5' 
			|| in == '6' || in == '7' || in == '8' || in == '9' || in == '0'){
			return 1;
		}
		else{
			return 0;
		}
}

//void callback(const std_msgs::UInt8MultiArray& msg) {
//	ROS_INFO("test: %i", msg.data[2]);
//}

int main(int argc, char **argv) {
	//Measure the time the algorithm takes
	beacon_order.data.clear();
	ros::init(argc, argv, "shortest_path");
	ros::NodeHandle n;
	pub = n.advertise<std_msgs::UInt8MultiArray>("shortest_path", 100);
	//ros::Subscriber sub = n.subscribe("shortest_path", 100, callback);
	ros::Rate loop_rate(1);

	timeval start, end;
	gettimeofday(&start, 0);
    
    //example output
	ofstream  fileout;
	fileout.open("beispiel.txt");
	fileout  << "Das  steht  jetzt  in der  Datei.";
	fileout.close ();
	//bool readIn = true; probably never used
	
	//yaml input
	string dateizeile [100];
	int help = 0;
	ifstream fin;
	fin.open("watchmen_route_0.yaml");
	while(!fin.eof() && help < 100){
		getline(fin,dateizeile[help]);
		cout  << dateizeile[help] << endl;
		help++;
	}
	fin.close();
	
	string helpstr;
	bool num;
	bool write;
	string help2;
	float_t numbers[100];
	int numCount = 0;
	int count;
	
	for(int i = 0; i < help; i++){
		cout << helpstr << endl;
		cout << "helpstr cleared" << endl;
		helpstr.clear();
		helpstr = dateizeile[i];
		count = 0;
		for(int j = 0; j < helpstr.length()+ 1; j++){
			if(isNumber(helpstr[j]) && helpstr[j-1] != 'e' && helpstr[j-1] != '_'){
				num = true;
				write = true;
				
			}
			if(helpstr[j] == ',' || helpstr[j] == ' ' || helpstr[j] == ']' ){
				num = false;
				count = 0;
			}
			if(num == true){
				help2[count] = helpstr[j];
				count++;
			}
			if(num == false && write == true){
				/*
				cout << help2 << "|" << endl;
				cout << ::atof(help2.c_str()) << "A" << endl;
				cout << static_cast<float_t>(::atof(help2.c_str())) << "B" << endl;
				* */
				numbers[numCount] = static_cast<float_t>(::atof(help2.c_str()));	
				write = false;
				
				for(int i = 0; i < 6; i++){
					help2[i] = '0';
				}

				numCount++;
			}
		}
	}
	for(int i = 0; i < numCount; i++){
		cout << "Beacon Nr. " << i/4 << ": " << numbers[i] << endl;
	}
	numCount--;
	
	cout << numbers[numCount-3] << endl;
	int bc = static_cast<int>(numbers[numCount-3]);
	bc = bc + 1;
	cout << "Beacon-Zahl (inkl. Basis): " << bc+1 << endl;
	
	
	//Calculate Distance of every beacon to every other beacon
	float_t beacDiff [bc+1][bc+1]; //Matrix mit Beaconanzahl * Beaconanzahl (+Basis)
	float_t beacDiff1d [(bc+1)*(bc+1)];
	float_t xDiff;
	float_t yDiff;
	
	for(int i = 0; i < bc + 1; i++){
		for(int j = 0; j < bc + 1; j++){
			if(i != j){
				xDiff = abs(numbers[i * 4 + 1]-numbers[j * 4 + 1]);
				yDiff = abs(numbers[i * 4 + 2]-numbers[j * 4 + 2]);
				beacDiff[i][j] = sqrtf((xDiff*xDiff)+(yDiff*yDiff));	
			}
			else{
				beacDiff[i][j] = 0;
			}
		}
	}
	//Write Matrix into linear vector for 'transmission'
	for(int i = 0; i < bc + 1; i++){
		for(int j = 0; j < bc + 1; j++){ 
			beacDiff1d[i*(bc+1) + j] = beacDiff[i][j];
			cout << "Distance from " << i << " to " << j << ": " << beacDiff[i][j] << endl;
		}
	}
	
	//Do magic. 🕶
	detMST(beacDiff1d, bc);
	

	//time measurement 2
	gettimeofday(&end, 0);
	long duration = (end.tv_sec-start.tv_sec)*1000000 + (end.tv_usec - start.tv_usec);
	//cout << start.tv_sec << ':' << start.tv_usec << endl;
	//cout << end.tv_sec << ':' << end.tv_usec << endl;
	cout << "🚀 Calculation took " << duration << " µseconds." << endl;
	cout << "🚀 That's " << duration/1000 << " milliseconds!" << endl;
	
	/*
    ros::init(argc, argv, "velpub");

    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<geometry_msgs::Twist>("amiro1/cmd_vel",1000);

    ros::Rate loop_rate(10);
    
    geometry_msgs::Twist msg;
    
    while(ros::ok()){
        msg.linear.x = 0.1;
        pub.publish(msg);
        ROS_INFO("%f", msg.linear.x);
        ros::spinOnce();
        loop_rate.sleep();
    }
    */
   	
		//pub.publish(beacon_order);
		//ros::spinOnce();
		//for(int i = 0; i <= 7; i++) {
		//	ROS_INFO("beacon_order[%i]: %i",i, beacon_order.data[i]);
		//}
		//loop_rate.sleep();

		
		return 0;
}



