#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <stack>
#include <string>

#include "Point.h"
#include "DCEL.hpp"



/*
  General helper functions
*/


// Get a vector with all half edges forming the face
std::vector<HalfEdge*> faceEdges(Face* f) {
    std::vector<HalfEdge*> halfEdges;
    
    HalfEdge* e = f->exteriorEdge;
    const HalfEdge* e_start = e;
    do {
        halfEdges.push_back(e);
        e = e->next;
    } while (e_start != e);
    
    return halfEdges;
}

// Flip halfedges direction
void flipEdge(HalfEdge* e) {
    Vertex* origin = e->origin;
    HalfEdge* prev = e->prev;
    e->origin = e->destination;
    e->destination = origin;
    e->prev = e->next;
    e->next = prev;
}

// Flip all halfedges of the face
void flipFace(Face* f) {
    for (auto e : faceEdges(f)) {
        flipEdge(e);
    }
}

// Recursively orient all faces the same
void orientFaces(Face* face) {

    std::stack<Face*> stack;
    std::set<Face*> visited;
    stack.push(face);
    visited.insert(face);

    while (!stack.empty()) 
    {
        Face* f = stack.top();
        stack.pop();

        for (auto e : faceEdges(f)) {
            if (visited.count(e->twin->incidentFace) == 0) {
                stack.push(e->twin->incidentFace);
                visited.insert(f);

                if (e->origin == e->twin->origin) {
                    flipFace(e->twin->incidentFace);
                }
            }
        }
    }
}


float signed_volume(const Point& a, const Point& b, const Point& c, const Point& d)
{
    // 1/6 * dot(a-d, cross(b-d, c-d)), ditch the 1/6, we need sign only
    Point
        ad = a - d,
        bd = b - d,
        cd = c - d;
    return ad.dot(bd.cross(cd));
}


bool is_opposite(const Point& a, const Point& b, const Point& v0, const Point& v1, const Point& v2)
{
    // signed volume multiple will be negative if the points are on the opposite sides
    return (0 > signed_volume(v0, v1, v2, a) * signed_volume(v0, v1, v2, b));
}


bool intersects(const Point& orig, const Point& dest, const Point& v0, const Point& v1, const Point& v2)
{
    // endpoints of the line are on opposite sides of the triangle
    // and three planes passing through the line and each vertex 
    // of the triangle have the two other vertices on opposite sides
    return (
        is_opposite(orig, dest, v0, v1, v2)
        && is_opposite(v0, v1, orig, dest, v2)
        && is_opposite(v1, v2, orig, dest, v0)
        );
}

bool intersects(const Point& orig, const Point& dest, const Face* f)
{
    Vertex* v_0 = f->exteriorEdge->origin;
    Vertex* v_1 = f->exteriorEdge->next->origin;
    Vertex* v_2 = f->exteriorEdge->prev->origin;
    Point v0{ v_0->x, v_0->y, v_0->z };
    Point v1{ v_1->x, v_1->y, v_1->z };
    Point v2{ v_2->x, v_2->y, v_2->z };
    return intersects(orig, dest, v0, v1, v2);
}


// Distance between points
double distance(Point a, Point b) {
    return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
}


// Get normal given 2 vertices
Point normal_vect(Vertex* v0, Vertex* v1, Vertex* v2)
{
    const Point a{
        v2->x - v0->x,
        v2->y - v0->y,
        v2->z - v0->z
    };

    const Point b{
        v1->x - v0->x,
        v1->y - v0->y,
        v1->z - v0->z
    };

    Point cross = a.cross(b);

    // normalise vector
    double len = distance(Point{ 0,0,0 }, cross);
    cross = cross / len;
    
    return cross;
}

Point normal_vect(Face* f)
{
    Vertex* v0 = f->exteriorEdge->origin;
    Vertex* v1 = f->exteriorEdge->next->origin;
    Vertex* v2 = f->exteriorEdge->prev->origin;
    return normal_vect(v0, v1, v2);
}


Point face_center(Vertex* v0, Vertex* v1, Vertex* v2)
{
    double x = v0->x + v1->x + v2->x;
    double y = v0->y + v1->y + v2->y;
    double z = v0->z + v1->z + v2->z;

    return Point{ x / 3, y / 3, z / 3 };
}

Point face_center(Face* f)
{
    Vertex* v0 = f->exteriorEdge->origin;
    Vertex* v1 = f->exteriorEdge->next->origin;
    Vertex* v2 = f->exteriorEdge->prev->origin;
    return face_center(v0, v1, v2);
}


Point normal_ray(Face* f, double length) {
    auto f_norm = normal_vect(f);
    auto f_center = face_center(f);
    Point ray_dest = f_center + f_norm * length;
    return ray_dest;
}



/*
  Main functions
*/

// 1.
bool importOBJ(DCEL & D, const char *file_in) {

    // Create map of OBJ vertex IDs to DCEL Vertex pointers
    // OBJ vertex IDs start at 1
    std::map<int, Vertex*> vmap;
    int v_index = 1;

    // Open the input file
    std::cout << "Reading file: " << file_in << "\n";
    std::ifstream infile(file_in, std::ifstream::in);
    if (!infile)
    {
        std::cerr << "Input file not found.\n";;
        return false;
    }

    // Iterate line by line
    std::string line;
    while (std::getline(infile, line))
    {
        if (line.substr(0, 2) == "v ")                  // Vertices
        {
            std::istringstream coords(line.substr(2));
            double x, y, z;    coords >> x; coords >> y; coords >> z;

            Vertex* v = D.createVertex(x, y, z);
            vmap.insert({ v_index, v });
            v_index++;
        }
        else if (line.substr(0, 2) == "f ")             // Faces
        {
            std::istringstream ids(line.substr(2));
            int v0, v1, v2;    ids >> v0; ids >> v1; ids >> v2;

            HalfEdge* e0 = D.createHalfEdge();
            HalfEdge* e1 = D.createHalfEdge();
            HalfEdge* e2 = D.createHalfEdge();
            Face* f0 = D.createFace();

            e0->origin = vmap[v0];
            e0->destination = vmap[v1];
            e0->next = e1;
            e0->prev = e2;
            e0->incidentFace = f0;

            e1->origin = vmap[v1];
            e1->destination = vmap[v2];
            e1->next = e2;
            e1->prev = e0;
            e1->incidentFace = f0;

            e2->origin = vmap[v2];
            e2->destination = vmap[v0];
            e2->next = e0;
            e2->prev = e1;
            e2->incidentFace = f0;

            f0->exteriorEdge = e0;
        }
    }

    // Close the input file
    infile.close();

    // Loop trough DCEL to assign correct twin edges
    for (const auto& he1 : D.halfEdges()) {
        for (const auto& he2 : D.halfEdges()) {

            if ((he1 != he2) && (
                    (he1->origin == he2->origin && he1->destination == he2->destination) ||
                    (he1->destination == he2->origin && he1->origin == he2->destination)
                )
            ){
                he1->twin = he2.get();
                he2->twin = he1.get();
                break;
            }
        }
    }

    return true;
}


// 2.
bool groupTriangles(DCEL & D, std::map<Face*, int> & facemap) {

    // Keep track of visited (n) / unvisited (0) faces
    std::map<Face*, int> visited;
    for (const auto & f : D.faces())
    {
        visited.insert({ f.get(), 0 });
    }

    std::stack<Face*> stack;
    int current_id = 1;
    bool done = false;

    while (!done) {
        done = true;
        
        // Check if there is some unvisited face
        Face* start;
        for (auto& f : visited) {
            if (f.second == 0) {
                start = f.first;
                done = false;
                break;
            }
        }

        if (done) break;

        // Add it to stack and infinite face holes
        stack.push(start);
        D.infiniteFace()->holes.push_back(start->exteriorEdge);

        while (!stack.empty())
        {
            Face* f = stack.top();
            stack.pop();
            visited[f] = current_id;

            for (auto e : faceEdges(f)) {
                if (visited[e->twin->incidentFace] == 0) {
                    stack.push(e->twin->incidentFace);
                }
            }
        }
        // Increase current ID and try again
        current_id++; 
    }

    facemap = visited;
    return true;
}


// 3.
bool orientMeshes(DCEL & D, std::map<Face*, int>& facemap) {

    // For each separate mesh
    for (const auto& e : D.infiniteFace()->holes) {
       
        // Get its first face that we can get
        Face* first = e->incidentFace;
        int mesh_id = facemap[first];

        // Shoot a ray along the face normal
        Point ray_orig = face_center(first);
        Point ray_dest = normal_ray(first, 9999);

        // Check the number of intersections
        int intersections = 0;
        for (const auto& f : D.faces())
        {
            if (facemap[f.get()] != mesh_id) continue;
            if (f.get() == first) continue;
            if (intersects(ray_orig, ray_dest, f.get()))
            {
                intersections++;
            }
        }
        // Correct orientation if even number of intersections happend
        if (intersections % 2 == 0) flipFace(first);

        // Recursively flip the rest based on the face
        orientFaces(first);
    }

    return true;
}


// 4.
bool mergeCoPlanarFaces(DCEL & D) {
  // to do

    return true;
}


// 5.
bool exportCityJSON(DCEL& D, std::map<Face*, int>& facemap, const char* file_out) {

    // Create map of DCEL Vertex pointers to their IDs for export
    // and reverse map of export IDs to the Vertex pointers
    // CityJSON vertex IDs start at 0
    std::map<Vertex*, int> vmap;
    std::map<int, Vertex*> rvmap;
    int v_index = 0;
    for (const auto& v : D.vertices()) {
        vmap.insert({ v.get(), v_index });
        rvmap.insert({ v_index, v.get() });
        v_index++;
    }

    // Open the file, write a generic CityJSON header
    std::cout << "Writing to file: " << file_out << "\n";
    std::ofstream file;
    file.open(file_out);
    file <<
        "{"
        "   \"type\":\"CityJSON\","
        "   \"version\":\"1.0\","
        "   \"CityObjects\":"
            "{";

    // Add CityObject for each separate mesh
    // For now just a single "building" to test
    int n_buildings = D.infiniteFace()->holes.size();
    
    for (int i = 1; i <= n_buildings; i++) {

        if (i > 1) file << ","; // Add comma in between

        file <<
            "\"Building " << i << "\": {"
            "   \"type\": \"Building\","
            "   \"attributes\": {},"
            "   \"geometry\":"
            "   [{"
            "       \"type\": \"MultiSurface\","
            "       \"lod\": 2,"
            "       \"boundaries\": [";

        // Add surface for each face
        bool first = true;
        for (const auto& f : D.faces()) {

            // Skip faces not belonging to this building
            if (facemap[f.get()] != i) continue;

            if (!first) file << ","; // Add comma in between surfaces
            first = false;

            file << "[";

            // Exterior surface boundary
            file << "[";
            HalfEdge* e = f->exteriorEdge;
            const HalfEdge* start = e;
            while (true) {
                int v_index = vmap[e->origin];
                file << v_index;
                e = e->next;
                if (e == start) break;
                file << ",";
            }
            file << "]";

            // Interior surface boundaries
            for (const auto& hole : f->holes)
            {
                file << ",[";
                HalfEdge* e = hole;
                const HalfEdge* start = e;
                while (true) {
                    int v_index = vmap[e->origin];
                    file << v_index;
                    e = e->next;
                    if (e == start) break;
                    file << ",";
                }
                file << "]";
            }

            file << "]";   // end one surface
        }

        // Close geometry and the whole building object
        file <<
                    "]"     // boundaries
                "}]"        // geometry
            "}";            // Building
    }
    // Close CityObjects and start with "vertices"
    file <<
        "},"
        "\"vertices\":[";

    char buffer[64];
    for (int i = 0; i < rvmap.size(); i++) {
        if (i > 0) file << ","; // Add comma in between
        Vertex* v = rvmap[i];
        sprintf(buffer, "[%f,%f,%f]",
                v->x, v->y, v->z);
        file << buffer;
    }

    // Close "vertices"
    file << "]";
    // Close the CityJSON object and the file itself
    file << "}";
    file.close();

    return true;
}



// forward declarations; these functions are given below main()
void printDCEL(DCEL & D);
bool testDCEL(DCEL& D);


int main(int argc, const char* argv[]) {
	const char* file_in = "bk_soup.obj";
	const char* file_out = "bk_out.json";

	// create an empty DCEL
	DCEL D;
    // empty map for group ids per face
    std::map<Face*, int> facemap;


	// 1. read the triangle soup from the OBJ input file and convert it to the DCEL,
	if (!importOBJ(D, file_in) || !testDCEL(D))
	{
		std::cerr << "File import failed.\n";
		return 1;
	};

	// 2. group the triangles into meshes, store in facemap
	if (!groupTriangles(D, facemap) || !testDCEL(D))
	{
		std::cerr << "Triangle grouping failed.\n";
		return 2;
	};

	// 3. determine the correct orientation for each mesh and ensure all its triangles 
	//    are consistent with this correct orientation (ie. all the triangle normals 
	//    are pointing outwards).
	if (!orientMeshes(D, facemap) || !testDCEL(D))
	{
		std::cerr << "Orientation check failed.\n";
		return 3;
	};

	// 4. merge adjacent triangles that are co-planar into larger polygonal faces.
	if (!mergeCoPlanarFaces(D) || !testDCEL(D))
	{
		std::cerr << "Co-planar face merge failed.\n";
		return 4;
	};

	// 5. write the meshes with their faces to a valid CityJSON output file.
	if (!exportCityJSON(D, facemap, file_out))
	{
		std::cerr << "File export failed.\n";
		return 5;
	};

	return 0;
}



bool testDCEL(DCEL& D) {

    // Quick check if there is an invalid element
    auto element = D.findInValid();
    if (element == nullptr) {
        // Beware that a 'valid' DCEL here only means there are no dangling links and no elimated elements.
        // There could still be problems like links that point to the wrong element.
        return true;
    }
    else {
        std::cout << "DCEL is NOT valid ---> ";
        std::cout << *element << "\n";
        return false;
    }
}


void printDCEL(DCEL & D) {

    testDCEL(D);

	// iterate all elements of the DCEL and print the info for each element
	const auto& vertices = D.vertices();
	const auto& halfEdges = D.halfEdges();
	const auto& faces = D.faces();
	std::cout << "DCEL has:\n";
	std::cout << " " << vertices.size() << " vertices:\n";
	for (const auto& v : vertices) {
		std::cout << "  * " << *v << "\n";
	}
	std::cout << " " << halfEdges.size() << " half-edges:\n";
	for (const auto& e : halfEdges) {
		std::cout << "  * " << *e << "\n";
	}
	std::cout << " " << faces.size() << " faces:\n";
	for (const auto& f : faces) {
		std::cout << "  * " << *f << "\n";
	}
}