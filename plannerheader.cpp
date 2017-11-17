#include <math.h>
#include "mex.h"
#include <list>
#include <queue>
#include <vector>
#include "plannerheader.hpp"

using namespace std;

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif


Graph::Graph(){	
	//mexPrintf("Graph Created\n");
}

Graph::Graph(int* blocksV, int numofblocks, int* trianglesV, int numoftriangles, int TableIndex, 
	int** onV_start, int onV_start_length, int* clearV_start, int numofclear_start, 
	int** onV_goal, int onV_goal_length, int* clearV_goal, int numofclear_goal){

	TableInd = TableIndex;

	for (int i = 0; i < numofblocks; i++)
		Blocks.push_back(blocksV[i]);

	for (int i = 0; i < numoftriangles; i++)
		Triangles.push_back(trianglesV[i]);

	for (int i = 0; i < onV_goal_length; i++){
		two_int a;
		a.a = onV_goal[i][0];
		a.b = onV_goal[i][1];
		GoalNode.on_literals.push_back(a);
	}

	for (int i = 0; i < numofclear_goal; i++)
		GoalNode.clear_literals.push_back(clearV_goal[i]);

	Node RootNode;
	RootNode.node_id = 1;
	RootNode.parent = NULL;
	for (int i = 0; i < onV_start_length; i++){
		two_int a;
		a.a = onV_start[i][0];
		a.b = onV_start[i][1];
		RootNode.on_literals.push_back(a);
	}

	for (int i = 0; i < numofclear_start; i++)
		RootNode.clear_literals.push_back(clearV_start[i]);

	RootNode.g_value = 0;
	RootNode.h_value = 0;

	RootNode.clear_literals.sort();
	RootNode.on_literals.sort();

	//RootNode.on_literals.sort();
	//RootNode.clear_literals.sort();
	Vertices.push_back(RootNode);
	open_set.push(&Vertices.back());
	//mexPrintf("Graph Created with custom constructor\n");
}

Graph::Graph(Graph* main_planner_graph, Node* CurrentNode){
	//mexPrintf("Heuristic Graph Created\n");
	main_graph = main_planner_graph;
	Node TempNode = *CurrentNode;
	TempNode.g_value = 0;
	TempNode.h_value = 0;
	TableInd = main_planner_graph->TableInd;
	GoalNode = main_planner_graph->GoalNode;
	Vertices.push_back(TempNode);
	open_set.push(&Vertices.back());
}

bool Graph::is_clear(Node* Current_State, int x){
	for (list<int>::iterator it = Current_State->clear_literals.begin(); it != Current_State->clear_literals.end(); it++){
		if (*it == x)
			return true;
	}
	return false;
}

bool Graph::is_on(Node* Current_State, int x, int y){
	for (list<two_int>::iterator it = Current_State->on_literals.begin(); it != Current_State->on_literals.end(); it++){
		if ((it->a == x) && (it->b == y))
			return true;
	}
	return false;
}

bool Graph::is_block(int x){
	for (list<int>::iterator it = Blocks.begin(); it != Blocks.end(); it++){
		if (*it == x)
			return true;
	}
	return false;
}

bool Graph::is_block_heuristic(Graph* Main_Graph, int x){
	for (list<int>::iterator it = Main_Graph->Blocks.begin(); it != Main_Graph->Blocks.end(); it++){
		if (*it == x)
			return true;
	}
	return false;
}

int Graph::block_is_on(Node* Current_State, int x){
	for (list<two_int>::iterator it = Current_State->on_literals.begin(); it != Current_State->on_literals.end(); it++){
		if ((it->a == x))
			return it->b;
	}
	return -1;
}

list<int> Graph::block_on_blocks(Node* Current_State, int x){
	list<int> temp_blocks;
	for (list<two_int>::iterator it = Current_State->on_literals.begin(); it != Current_State->on_literals.end(); it++){
		if ((it->a == x))
			temp_blocks.push_back(it->b);
	}
	return temp_blocks;
}


bool Graph::are_nodes_equal(Node* node1, Node* node2){
	if(node1->clear_literals.size() != node2->clear_literals.size())
		return false;

	if(node1->on_literals.size() != node2->on_literals.size())
		return false;

	// node1->clear_literals.sort();
	// node2->clear_literals.sort();

	list<int>::iterator it1,it2;
	it1 = node1->clear_literals.begin();
	it2 = node2->clear_literals.begin();

	while(it1 != node1->clear_literals.end()){
		if(*it1 != *it2)
			return false;

		it1++;
		it2++;
	}

	// node1->on_literals.sort();
	// node2->on_literals.sort();

	list<two_int>::iterator it3,it4;
	it3 = node1->on_literals.begin();
	it4 = node2->on_literals.begin();

	while(it3 != node1->on_literals.end()){
		if((it3->a != it4->a) || (it3->b != it4->b))
			return false;

		it3++;
		it4++;
	}
	return true;
}

bool Graph::is_Node_in_Graph(Node* Current_Node, Node* Original_Node){
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		if(are_nodes_equal(Current_Node, &(*it))){
			Original_Node = &(*it);
			return true;
		}
	}
	return false;

}

int Graph::heuristic_for_heuristic(Node* Cur_Node, Node* Goal_Node){

	int counter = 0;

	for (list<int>::iterator it = Goal_Node->clear_literals.begin(); it != Goal_Node->clear_literals.end(); it++){
		if(is_clear(Cur_Node,*it))
			counter++;
	}

	for (list<two_int>::iterator it = Goal_Node->on_literals.begin(); it != Goal_Node->on_literals.end(); it++){
		if(is_on(Cur_Node, it->a, it->b))
			counter++;
	}

	return (Goal_Node->on_literals.size() + Goal_Node->clear_literals.size() - counter);
}

bool Graph::is_goal_in_node(Node* Cur_Node, Node* Goal_Node){

	for (list<int>::iterator it = Goal_Node->clear_literals.begin(); it != Goal_Node->clear_literals.end(); it++){
		if(!is_clear(Cur_Node,*it))
			return false;
	}

	for (list<two_int>::iterator it = Goal_Node->on_literals.begin(); it != Goal_Node->on_literals.end(); it++){
		if(!is_on(Cur_Node, it->a, it->b))
			return false;
	}

	return true;
}

void Graph::delete_on(Node* Current_State, int x, int y){
	for (list<two_int>::iterator it = Current_State->on_literals.begin(); it != Current_State->on_literals.end(); it++){
		if (it->a == x && it->b == y){
			Current_State->on_literals.erase(it);
			return;
		}
	}
}

void Graph::add_on(Node* Current_State, int x, int y){
	for (list<two_int>::iterator it = Current_State->on_literals.begin(); it != Current_State->on_literals.end(); it++){
		if (it->a == x && it->b == y){
			return;
		}
	}

	two_int temp;
	temp.a = x;
	temp.b = y;
	Current_State->on_literals.push_back(temp);	
}


void Graph::move_to(Node* Current_State, int x, int y, int z){
	// Preconditions
	if(!is_clear(Current_State,x))
		return;
	if(!is_clear(Current_State,z))
		return;
	if(!is_block(z))
		return;
	if(!is_on(Current_State,x,y))
		return;

	//Create a node for the new state
	Node New_node;
	New_node = *Current_State;
	
	New_node.parent = Current_State;

	//Apply effects
	delete_on(&New_node, x, y);
	New_node.clear_literals.remove(z);
	if(y != TableInd){
		New_node.clear_literals.push_back(y);
	}
	two_int temp;
	temp.a = x;
	temp.b = z;
	New_node.on_literals.push_back(temp);
	
	New_node.g_value = Current_State->g_value + 1;
	New_node.h_value = Get_Heuristic(&New_node);
	//New_node.h_value = 0;
	New_node.action_index = 0;
	New_node.action_blocks[0] = x;
	New_node.action_blocks[1] = y;
	New_node.action_blocks[2] = z;

	New_node.clear_literals.sort();
	New_node.on_literals.sort();

	Node* TempNode;

	if (!is_Node_in_Graph(&New_node, TempNode)){
		New_node.node_id = Vertices.size() + 1;
	//Add the node to the graph
		Vertices.push_back(New_node);
		open_set.push(&Vertices.back());
	}

	else{
		if (!TempNode->is_closed){
			if(New_node.g_value < TempNode->g_value)
				TempNode->g_value = New_node.g_value;
		}
	}
}

void Graph::move_to_heuristic(Node* Current_State, int x, int y, int z){
	// Preconditions
	if(!is_clear(Current_State,x))
		return;
	if(!is_clear(Current_State,z))
		return;
	if(!is_block_heuristic(main_graph,z))
		return;
	if(!is_on(Current_State,x,y))
		return;

	//Create a node for the new state
	Node New_node;
	New_node = *Current_State;
	New_node.node_id = Vertices.size() + 1;
	New_node.parent = Current_State;

	//Apply effects without deletions
	if(y != TableInd){
		New_node.clear_literals.remove(y);
		New_node.clear_literals.push_back(y);
	}
	add_on(&New_node, x, z);
	New_node.g_value = Current_State->g_value + 1;
	//print_Node_Info(&GoalNode);
	New_node.h_value = heuristic_for_heuristic(&New_node, &GoalNode);
	//Add the node to the graph
	Vertices.push_back(New_node);
	open_set.push(&Vertices.back());
}

void Graph::move_to_table(Node* Current_State, int x, int y){
	// Preconditions
	if(!is_clear(Current_State,x))
		return;
	if(!is_block(y))
		return;
	if(!is_on(Current_State,x,y))
		return;

	//Create a node for the new state
	Node New_node;
	New_node = *Current_State;
	New_node.parent = Current_State;

	//Apply effects
	delete_on(&New_node, x, y);
	New_node.clear_literals.push_back(y);
	two_int temp;
	temp.a = x;
	temp.b = TableInd;
	New_node.on_literals.push_back(temp);
	New_node.g_value = Current_State->g_value + 1;
	New_node.h_value = Get_Heuristic(&New_node);
	//New_node.h_value = 0;
	New_node.action_index = 1;
	New_node.action_blocks[0] = x;
	New_node.action_blocks[1] = y;
	New_node.action_blocks[2] = TableInd;

	New_node.clear_literals.sort();
	New_node.on_literals.sort();

	Node* TempNode;

	if (!is_Node_in_Graph(&New_node, TempNode)){
		New_node.node_id = Vertices.size() + 1;
	//Add the node to the graph
		Vertices.push_back(New_node);
		open_set.push(&Vertices.back());
	} 
	else{
		if (!TempNode->is_closed){
			if(New_node.g_value < TempNode->g_value)
				TempNode->g_value = New_node.g_value;
		}
	}

}

void Graph::move_to_table_heuristic(Node* Current_State, int x, int y){
	// Preconditions
	if(!is_clear(Current_State,x))
		return;
	if(!is_block_heuristic(main_graph,y))
		return;
	if(!is_on(Current_State,x,y))
		return;

	//Create a node for the new state
	Node New_node;
	New_node = *Current_State;
	New_node.node_id = Vertices.size() + 1;
	New_node.parent = Current_State;

	//Apply effects without deletions
	New_node.clear_literals.remove(y);
	New_node.clear_literals.push_back(y);
	add_on(&New_node, x, TableInd);
	New_node.g_value = Current_State->g_value + 1;
	New_node.h_value = heuristic_for_heuristic(&New_node, &GoalNode);
	//Add the node to the graph
	Vertices.push_back(New_node);
	open_set.push(&Vertices.back());
}

void Graph::Get_Succesors(Node* Current_Node){
	for(int i=0; i < TableInd; i++){
		for(int j=0; j <= TableInd; j++){
			for(int k=0; k < TableInd; k++){
				if(i != j){
					if (i != k){
						move_to(Current_Node, i, j, k);
					}
				}
			}
			if(i != j){
				if(j != TableInd){
					move_to_table(Current_Node, i, j);
				}
			}
		}
	}
}

void Graph::Get_Succesors_heuristic(Node* Current_Node){
	for(int i=0; i< (TableInd); i++){
		for(int j=0; j <= TableInd; j++){
			for(int k=0; k < TableInd; k++){
				if(i != j){
					if (i != k){
						move_to_heuristic(Current_Node, i, j, k);
					}
				}
			}
			if(i != j){
				if(j != TableInd){
					move_to_table_heuristic(Current_Node, i, j);
				}
			}
		}
	}
}

int Graph::Get_Heuristic(Node* Current_Node){
	Graph Heuristic_Graph(this, Current_Node);
	Node* Cur_Node;

	int counter = 0;

	// Dijkstra to find the heuristic values
	while (!Heuristic_Graph.open_set.empty()){
		Cur_Node = Heuristic_Graph.open_set.top();
		if (Heuristic_Graph.is_goal_in_node(Cur_Node, &GoalNode)){
			return Cur_Node->g_value;
		}
		//Cur_Node->is_closed = true;
		Heuristic_Graph.open_set.pop();
		Heuristic_Graph.Get_Succesors_heuristic(Cur_Node);
	}

	return -1;
}

void Graph::print_Node_Info(Node* CurrentNode){
	mexPrintf("Printing Node info \n -------\n");
	mexPrintf("Printing On literals for Node %d\n",CurrentNode->node_id);
	for (list<two_int>::iterator it1 = CurrentNode->on_literals.begin(); it1 != CurrentNode->on_literals.end(); it1++){
		mexPrintf("%d on %d\n",it1->a, it1->b);
	}
	mexPrintf("Printing Clear literals for Node %d\n",CurrentNode->node_id);
	for (list<int>::iterator it2 = CurrentNode->clear_literals.begin(); it2 != CurrentNode->clear_literals.end(); it2++){
		mexPrintf("%d ",*it2);
	}
	mexPrintf("\n");
	mexPrintf("g_value: %d\n",CurrentNode->g_value);
	mexPrintf("h_value: %d\n",CurrentNode->h_value);
	if (CurrentNode->parent != NULL)
		mexPrintf("Parent Id: %d\n",CurrentNode->parent->node_id);
	mexPrintf("\n");
}

void Graph::print_Graph_Info(){
	mexPrintf("Printing Graph info \n -------\n");
	mexPrintf("Number of nodes in Graph %d\n", Vertices.size());
	for (list<Node>::iterator it1 = Vertices.begin(); it1 != Vertices.end(); it1++){
		print_Node_Info(&(*it1));		
	}
	return;
}

Graph::~Graph(){
	//mexPrintf("Graph Destroyed\n");
}