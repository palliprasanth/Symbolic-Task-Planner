#ifndef PLANNER_HEADER_H   
#define PLANNER_HEADER_H

using namespace std;

typedef struct Node Node;

typedef struct two_int two_int;

struct two_int{
	int a;
	int b;

	bool operator<(const two_int& temp_two_int) const
	{
		return (a < temp_two_int.a);
	}

};

struct Node{
	int node_id;
	int action_index;
	Node* parent;
	list<two_int> on_literals;
	list<int> clear_literals;
	int g_value;
	int h_value;
	bool is_closed;
	int action_blocks[3];
};

class myComparator
{
public:
	int operator() (const Node* p1, const Node* p2)
	{
		return (p1->g_value + p1->h_value) > (p2->g_value + p2->h_value);
	}
};

class Graph{
public:
	list<int> Blocks;
	list<int> Triangles;
	int TableInd;
	Node GoalNode;

	list<Node> Vertices;
	std::priority_queue<Node*, vector<Node*>, myComparator> open_set;
	Graph* main_graph; // We use this only if the graph is a heuristic graph

	// Constructors
	Graph();
	Graph(int* , int , int* , int , int , int** , int , int* , int , int** , int , int* , int );
	Graph(Graph* , Node*);

	bool is_clear(Node*, int);
	bool is_on(Node*, int, int);
	bool is_block(int);
	bool is_block_heuristic(Graph*, int);

	bool are_nodes_equal(Node*, Node*);
	bool is_goal_in_node(Node*, Node*);
	bool is_Node_in_Graph(Node*, Node*);

	int block_is_on(Node*, int);
	list<int> block_on_blocks(Node*, int);

	void add_on(Node*, int, int);
	void delete_on(Node*, int, int);

	void move_to(Node*, int, int, int);
	void move_to_heuristic(Node*, int, int, int);
	void move_to_table(Node*, int, int);
	void move_to_table_heuristic(Node*, int, int);

	void Get_Succesors(Node*); // Adds valid succesors to the list of children
	void Get_Succesors_heuristic(Node*);

	int Get_Heuristic(Node*);
	int heuristic_for_heuristic(Node*, Node*);

	void print_Node_Info(Node* );
	void print_Graph_Info();

	// Deconstructors
	~Graph();
};

#endif 