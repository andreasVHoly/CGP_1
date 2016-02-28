//
// Mesh
//

#include "mesh.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <list>
#include <sys/stat.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtx/intersect.hpp>
#include <unordered_map>


using namespace std;
using namespace cgp;

GLfloat stdCol[] = {0.7f, 0.7f, 0.75f, 0.4f};
const int raysamples = 5;

bool Sphere::pointInSphere(cgp::Point pnt)
{
    cgp::Vector delvec;

    delvec.diff(c, pnt);
    if(delvec.sqrdlength() < r*r)
        return true;
    else
        return false;
}

bool Mesh::findVert(cgp::Point pnt, int &idx)
{
    bool found = false;
    int i = 0;

    idx = -1;
    // linear search of vertex list
    while(!found && i < (int) verts.size())
    {
        if(verts[i] == pnt)
        {
            found = true;
            idx = i;
        }
        i++;
    }
    return found;
}

bool Mesh::findEdge(vector<Edge> edges, Edge e, int &idx)
{
    bool found = false;
    int i = 0;

    idx = -1;
    // linear search of edge list
    while(!found && i < (int) edges.size())
    {
        if( (edges[i].v[0] == e.v[0] && edges[i].v[1] == e.v[1]) || (edges[i].v[1] == e.v[0] && edges[i].v[0] == e.v[1]) )
        {
            found = true;
            idx = i;
        }
        i++;
    }
    return found;
}



bool Mesh::findEdge(vector<Edge> edges, Edge e)
{
    bool found = false;
    int i = 0;

    // linear search of edge list
    while(!found && i < (int) edges.size())
    {
        if( (edges[i].v[0] == e.v[0] && edges[i].v[1] == e.v[1]) || (edges[i].v[1] == e.v[0] && edges[i].v[0] == e.v[1]) )
        {
            found = true;
        }
        i++;
    }
    return found;
}




long Mesh::hashVert(cgp::Point pnt)
{
    long x, y, z;
    float range = 2500.0f;
    long lrangesq, lrange = 2500;

    lrangesq = lrange * lrange;

    // discretise vertex within bounds of the enclosing bounding box
    x = (long) (((pnt.x - bbox.min.x) * range) / bbox.diagLen()) * lrangesq;
    y = (long) (((pnt.y - bbox.min.y) * range) / bbox.diagLen()) * lrange;
    z = (long) (((pnt.z - bbox.min.z) * range) / bbox.diagLen());
    return x+y+z;
}

void Mesh::mergeVerts()
{
    vector<cgp::Point> cleanverts;
    long key;
    int i, p, hitcount = 0;
    // use hashmap to quickly look up vertices with the same coordinates
    std::unordered_map<long, int> idxlookup; // key is concatenation of vertex position, value is index into the cleanverts vector
    //cgp::BoundBox bbox;

    // construct a bounding box enclosing all vertices
    for(i = 0; i < (int) verts.size(); i++)
        bbox.includePnt(verts[i]);

    // remove duplicate vertices
    for(i = 0; i < (int) verts.size(); i++)
    {
        key = hashVert(verts[i]);
        if(idxlookup.find(key) == idxlookup.end()) // key not in map
        {
            idxlookup[key] = (int) cleanverts.size(); // put index in map for quick lookup
            cleanverts.push_back(verts[i]);
        }
        else
        {
            hitcount++;
        }
    }
    cerr << "num duplicate vertices found = " << hitcount << " of " << (int) verts.size() << endl;
    cerr << "clean verts = " << (int) cleanverts.size() << endl;
    no_vert_clean = cleanverts.size();
    no_vert_dirty = verts.size();

    //cerr << "bbox min = " << bbox.min.x << ", " << bbox.min.y << ", " << bbox.min.z << endl;
    //cerr << "bbox max = " << bbox.max.x << ", " << bbox.max.y << ", " << bbox.max.z << endl;
    //cerr << "bbox diag = " << bbox.diagLen() << endl;


    // re-index triangles
    for(i = 0; i < (int) tris.size(); i++)
        for(p = 0; p < 3; p++)
        {
            key = hashVert(verts[tris[i].v[p]]);
            if(idxlookup.find(key) != idxlookup.end())
                tris[i].v[p] = idxlookup[key];
            else
                cerr << "Error Mesh::mergeVerts: vertex not found in map" << endl;

        }

    verts.clear();
    verts = cleanverts;
}

void Mesh::deriveVertNorms()
{
    vector<int> vinc; // number of faces incident on vertex
    int p, t;
    cgp::Vector n;

    // init structures
    for(p = 0; p < (int) verts.size(); p++)
    {
        vinc.push_back(0);
        norms.push_back(cgp::Vector(0.0f, 0.0f, 0.0f));
    }

    // accumulate face normals into vertex normals
    for(t = 0; t < (int) tris.size(); t++)
    {
        n = tris[t].n; n.normalize();
        for(p = 0; p < 3; p++)
        {
            norms[tris[t].v[p]].add(n);
            vinc[tris[t].v[p]]++;
        }
    }

    // complete average
    for(p = 0; p < (int) verts.size(); p++)
    {
        norms[p].mult(1.0f/((float) vinc[p]));
        norms[p].normalize();
    }
}

void Mesh::deriveFaceNorms()
{
    int t;
    cgp::Vector evec[2];

    for(t = 0; t < (int) tris.size(); t++)
    {
        // right-hand rule for calculating normals, i.e. counter-clockwise winding from front on vertices
        evec[0].diff(verts[tris[t].v[0]], verts[tris[t].v[1]]);
        evec[1].diff(verts[tris[t].v[0]], verts[tris[t].v[2]]);
        evec[0].normalize();
        evec[1].normalize();
        tris[t].n.cross(evec[0], evec[1]);
        tris[t].n.normalize();
    }

}

void Mesh::buildTransform(glm::mat4x4 &tfm)
{
    glm::mat4x4 idt;

    idt = glm::mat4(1.0f);
    tfm = glm::translate(idt, glm::vec3(trx.i, trx.j, trx.k));
    tfm = glm::rotate(tfm, zrot, glm::vec3(0.0f, 0.0f, 1.0f));
    tfm = glm::rotate(tfm, yrot, glm::vec3(0.0f, 1.0f, 0.0f));
    tfm = glm::rotate(tfm, xrot, glm::vec3(1.0f, 0.0f, 0.0f));
    tfm = glm::scale(tfm, glm::vec3(scale));
}

Mesh::Mesh(){
    col = stdCol;
    scale = 1.0f;
    xrot = yrot = zrot = 0.0f;
    trx = cgp::Vector(0.0f, 0.0f, 0.0f);
}

Mesh::~Mesh(){
    clear();
}

void Mesh::clear(){
    verts.clear();
    tris.clear();
    geom.clear();
    col = stdCol;
    scale = 1.0f;
    xrot = yrot = zrot = 0.0f;
    trx = cgp::Vector(0.0f, 0.0f, 0.0f);
    for(int i = 0; i < (int) boundspheres.size(); i++)
        boundspheres[i].ind.clear();
    boundspheres.clear();
    edges.clear();
    //added
    edges.clear();
}

bool Mesh::genGeometry(View * view, ShapeDrawData &sdd){
    vector<int> faces;
    int t, p;
    glm::mat4x4 tfm;

    geom.clear();
    geom.setColour(col);

    // transform mesh data structures into a form suitable for rendering
    // by flattening the triangle list
    for(t = 0; t < (int) tris.size(); t++)
        for(p = 0; p < 3; p++)
            faces.push_back(tris[t].v[p]);

    // construct transformation matrix
    buildTransform(tfm);
    geom.genMesh(&verts, &norms, &faces, tfm);

    /*
    // generate geometry for acceleration spheres for testing
    for(p = 0; p < (int) boundspheres.size(); p++)
    {
        glm::mat4x4 idt;

        idt = glm::mat4(1.0f);
        tfm = glm::translate(idt, glm::vec3(trx.i+boundspheres[p].c.x, trx.j+boundspheres[p].c.y, trx.k+boundspheres[p].c.z));
        tfm = glm::rotate(tfm, zrot, glm::vec3(0.0f, 0.0f, 1.0f));
        tfm = glm::rotate(tfm, yrot, glm::vec3(0.0f, 1.0f, 0.0f));
        tfm = glm::rotate(tfm, xrot, glm::vec3(1.0f, 0.0f, 0.0f));
        tfm = glm::scale(tfm, glm::vec3(scale));
        geom.genSphere(boundspheres[p].r, 20, 20, tfm);
    }*/

    // bind geometry to buffers and return drawing parameters, if possible
    if(geom.bindBuffers(view))
    {
        sdd = geom.getDrawParameters();
        return true;
    }
    else
       return false;
}

void Mesh::boxFit(float sidelen){
    cgp::Point pnt;
    cgp::Vector shift, diag, halfdiag;
    float scale;
    int v;
    cgp::BoundBox bbox;

    // calculate current bounding box
    for(v = 0; v < (int) verts.size(); v++)
        bbox.includePnt(verts[v]);

    if((int) verts.size() > 0)
    {
        // calculate translation necessary to move center of bounding box to the origin
        diag = bbox.getDiag();
        shift.pntconvert(bbox.min);
        halfdiag = diag; halfdiag.mult(0.5f);
        shift.add(halfdiag);
        shift.mult(-1.0f);

        // scale so that largest side of bounding box fits sidelen
        scale = max(diag.i, diag.j); scale = max(scale, diag.k);
        scale = sidelen / scale;

        // shift center to origin and scale uniformly
        for(v = 0; v < (int) verts.size(); v++)
        {
            pnt = verts[v];
            shift.pntplusvec(pnt, &pnt);
            pnt.x *= scale; pnt.y *= scale; pnt.z *= scale;
            verts[v] = pnt;
        }
    }
}

bool Mesh::readSTL(string filename)
{


    cout << "************************************" << filename << "************************************" << std::endl;
    ifstream infile;
    char * inbuffer;
    struct stat results;
    int insize, inpos, numt, t, i;
    cgp::Point vpos;
    Triangle tri;

    // assumes binary format STL file
    infile.open((char *) filename.c_str(), ios_base::in | ios_base::binary);
    if(infile.is_open())
    {
        clear();

        // get the size of the file
        stat((char *) filename.c_str(), &results);
        insize = results.st_size;

        // put file contents in buffer
        inbuffer = new char[insize];
        infile.read(inbuffer, insize);
        if(!infile) // failed to read from the file for some reason
        {
            cerr << "Error Mesh::readSTL: unable to populate read buffer" << endl;
            return false;
        }

        // interpret buffer as STL file
        if(insize <= 84)
        {
            cerr << "Error Mesh::readSTL: invalid STL binary file, too small" << endl;
            return false;
        }

        inpos = 80; // skip 80 character header
        if(inpos+4 >= insize){ cerr << "Error Mesh::readSTL: malformed header on stl file" << endl; return false; }
        numt = (int) (* ((long *) &inbuffer[inpos]));
        inpos += 4;

        t = 0;

        // triangle vertices have consistent outward facing clockwise winding (right hand rule)
        while(t < numt) // read in triangle data
        {
            // normal
            if(inpos+12 >= insize){ cerr << "Error Mesh::readSTL: malformed stl file" << endl; return false; }
            // IEEE floating point 4-byte binary numerical representation, IEEE754, little endian
            tri.n = cgp::Vector((* ((float *) &inbuffer[inpos])), (* ((float *) &inbuffer[inpos+4])), (* ((float *) &inbuffer[inpos+8])));
            inpos += 12;

            // vertices
            for(i = 0; i < 3; i++)
            {
                if(inpos+12 >= insize){ cerr << "Error Mesh::readSTL: malformed stl file" << endl; return false; }
                vpos = cgp::Point((* ((float *) &inbuffer[inpos])), (* ((float *) &inbuffer[inpos+4])), (* ((float *) &inbuffer[inpos+8])));
                tri.v[i] = (int) verts.size();
                verts.push_back(vpos);
                inpos += 12;
            }




            tris.push_back(tri);
            t++;
            inpos += 2; // handle attribute byte count - which can simply be discarded
        }

        // tidy up
        delete inbuffer;
        infile.close();

        cerr << "num vertices = " << (int) verts.size() << endl;
        cerr << "num triangles = " << (int) tris.size() << endl;

        no_trinagles = tris.size();
        no_vert_dirty = verts.size();
        no_edges_dirty = edges.size();


        // STL provides a triangle soup so merge vertices that are coincident
        mergeVerts();
        // normal vectors at vertices are needed for rendering so derive from incident faces
        deriveVertNorms();
        /*if(basicValidity())
            cerr << "loaded file has basic validity" << endl;
        else
            cerr << "loaded file does not pass basic validity" << endl;*/
    }
    else
    {
        cerr << "Error Mesh::readSTL: unable to open " << filename << endl;
        return false;
    }
    return true;
}

bool Mesh::writeSTL(string filename)
{
    ofstream outfile;
    int t, p, numt;
    unsigned short val;

    outfile.open((char *) filename.c_str(), ios_base::out | ios_base::binary);
    if(outfile.is_open())
    {
        outfile.write("File Generated by Tesselator. Binary STL", 80); // skippable header
        numt = (int) tris.size();
        outfile.write((char *) &numt, 4); // number of triangles

        for(t = 0; t < numt; t++)
        {
            // normal
            outfile.write((char *) &tris[t].n.i, 4);
            outfile.write((char *) &tris[t].n.j, 4);
            outfile.write((char *) &tris[t].n.k, 4);

            // triangle vertices
            for(p = 0; p < 3; p++)
            {
                outfile.write((char *) &verts[tris[t].v[p]].x, 4);
                outfile.write((char *) &verts[tris[t].v[p]].y, 4);
                outfile.write((char *) &verts[tris[t].v[p]].z, 4);
            }

            // attribute byte count - null
            val = 0;
            outfile.write((char *) &val, 2);
        }

        // tidy up
        outfile.close();
    }
    else
    {
        cerr << "Error Mesh::writeSTL: unable to open " << filename << endl;
        return false;
    }
    return true;
}



int Mesh::prepareEdges(){
    //build all the edges including duplicate ones
    buildDirtyEdges();
    //sort the edge list to only contain unique edges
    return hashEdgeSort();
}

bool Mesh::basicValidity()
{
    //BUILDING THE UNIQUE EDGE LIST
    prepareEdges();
    std::cout << std::endl;

    //check eulers characteristic
    if (!checkEulerCharBool()){
        return false;
    }
    std::cout << std::endl;
    //2. no dangling vertices
    if (!danglingVerticeCheck()){
        return false;
    }
    std::cout << std::endl;
    //3. edge indices within bounds of the vertex list
    if (!vertexBoundsTest()){
        return false;
    }

    //if we have basic validity we would have reached this point
    //if not this method would have returned false before this point
    //if we pass maifold validity we have passed all tests
    //if not we return false

    /*if (!manifoldValidity()){
        return false;
    }
    else{

    }*/
    return true;
}

bool Mesh::manifoldValidity()
{
    std::cout << std::endl;
    //1. check the model to see if it is closed
    if (!closedTest()){
        return false;
    }
    std::cout << std::endl;
    //2. check to see if the model is orientable
    if (!orientableTest()){
        return false;
    }
    std::cout << std::endl;
    //3. check to see if the model is manifold
    if (!manifoldTest()){
        return false;
    }

    return true;
}


bool Mesh::checkEulerCharBool(){
    std::cout << "Checking Euler's characteristic..." << std::endl;

    //1. report euler's characteristic,
    std::cout << "Eulers characteristic: V - E + T = 2 - 2G" << std::endl;
    std::cout << "V = " << no_vert_clean << std::endl;
    std::cout << "E = " << no_edges_clean << std::endl;
    std::cout << "T = " << no_trinagles << std::endl;

    int result =  no_vert_clean - no_edges_clean + no_trinagles;
    std::cout << "Eulers characteristic Equation = " << result  << std::endl;
    return true;
}

int Mesh::checkEulerCharInt(){
    std::cout << "Checking Euler's characteristic..." << std::endl;

    //1. report euler's characteristic,
    std::cout << "Eulers characteristic: V - E + T = 2 - 2G" << std::endl;
    std::cout << "V = " << no_vert_clean << std::endl;
    std::cout << "E = " << no_edges_clean << std::endl;
    std::cout << "T = " << no_trinagles << std::endl;

    int result =  no_vert_clean - no_edges_clean + no_trinagles;
    std::cout << "Eulers characteristic Equation = " << result  << std::endl;

    return result;
}

bool Mesh::danglingVerticeCheck(){
    std::cout << "Checking for dangling vetices..." << std::endl;
    std::vector<int> dangling(no_vert_clean,0);

    //loop through triangle list
    for (int k = 0; k < tris.size(); k++){
        //go through each vertex in the triangles
        for (int j = 0; j < 3; j++){
            dangling[tris[k].v[j]]++;
        }
    }

    int danglingSize = dangling.size();
    int counter = dangling.size() - verts.size();

    if (counter > 0){
        std::cout << "Dangling vertices found: " << counter << std::endl;
        return false;
    }
    std::cout << "No dangling vertices found..." << std::endl;
    return true;
}

bool Mesh::vertexBoundsTest(){
    //we need to check if each vertex index stored in the edge table is within the bounds of the vertex data structure
    //do do this we need to check each vertex index to see if it is >= 0 and < than the size of the DS
    std::cout << "Checking if edge indices are within vertex list bounds..." << std::endl;

    int edgeSize = edges.size();
    int vertSize = verts.size();

    for (int x = 0; x < edgeSize; x++){
        if (edges[x].v[0] >= 0 && edges[x].v[0] < vertSize){
            continue;
        }
        else{
            std::cout << "Error: Index out of bounds" << std::endl;
            return false;
        }
    }
    std::cout << "No indices are out of bounds..." << std::endl;
    return true;
}

bool Mesh::manifoldTest(){
    std::cout << "Checking manifold validity of model..." << std::endl;
    bool result = true;

    //here we are using the property of:
        //if a vertex is connected to at least 3 edges
        //the number of triangles surrounding the vertex is equal to the number of edges connected to it
        //thus we can check that if for each vertex the number of connected edges = the number of triangles that vertex belongs to

    //for the 2 vectors below each unique vertex has a position in this vector based on its index into the verts vector
    //this vector holds the triangles for each vertex,
    std::vector<std::vector<Triangle>> triangleCont(verts.size());
    //this vector holds the edges for each vertex
    std::vector<std::vector<Edge>> edgeCont(verts.size());

    //we loop through the triangle list to get the triangles connected to each unique vertex
    for (int i = 0; i < tris.size(); i++){
        //get the 3 vertice indecies
        //these serve as unique identifiers into the vector
        int v0 = tris[i].v[0];
        int v1 = tris[i].v[1];
        int v2 = tris[i].v[2];

        //we assign the current triangle to these vertcies as they are part of it
        triangleCont[v0].push_back(tris[i]);
        triangleCont[v1].push_back(tris[i]);
        triangleCont[v2].push_back(tris[i]);
    }

    //we loop through all the unique edges to get the vertcies and its connected edges
    for (int j = 0; j < edges.size(); j++){
        //get the 2 vertice indecies
        //these serve as unique identifiers into the vector
        int v0 = edges[j].v[0];
        int v1 = edges[j].v[1];

        //we assign the current edge to these vertcies as they are part of it
        edgeCont[v0].push_back(edges[j]);
        edgeCont[v1].push_back(edges[j]);
    }

    //now using the property explained earlier
    //loop through both vectors and for each vertex compare the sizes
    //if they are equal the vertex is surrounded by a closed fan
    for(int x = 0; x < verts.size(); x++){
        if (edgeCont[x].size() != triangleCont[x].size()){
            result = false;
            break;
        }
    }

    if(result){
        std::cout << "Model is 2-Manifold..." << std::endl;
    }else{
        std::cout << "Error: model is not 2 manifold" << std::endl;
    }


    return result;
}

bool Mesh::orientableTest(){
    std::cout << "Checking if model is orientable..." << std::endl;

    //this number is calcauated in the hashEdgeSort method
    if (windingError > 0){
        std::cout << "Model is not orientable... ->" << windingError << " error found" << std::endl;
        return false;
    }
    std::cout << "Model is orientable..." << std::endl;
    return true;
}

void Mesh::buildDirtyEdges(){

    edges.clear();
    //we loop through the triangles and create a vector with all the edges in it, including duplicates
    for (int i = 0; i < (int) tris.size(); i++){
        Edge e1 = Edge(tris[i].v[0],tris[i].v[1]);
        Edge e2 = Edge(tris[i].v[1],tris[i].v[2]);
        Edge e3 = Edge(tris[i].v[2],tris[i].v[0]);

        edges.push_back(e1);
        edges.push_back(e2);
        edges.push_back(e3);

    }
}

bool Mesh::closedTest(){
    std::cout << "Checking if the model is closed..." << endl;
    int closeCounter = 0;
    for(auto i = edgelookup.begin(); i != edgelookup.end(); i++){
        //we need to go through all edges
        //then we need to check that each edge in the triangle list
        //if it shows up twice, the model is closed
        //we do this by checking that the counter we use is the same size as the size of destination vertices attached to each vertex
        if (i->second[0] != i->second.size()-1){
            //we subtract the differnce,
            //if this counter is bigger than 0, we have a non-closed model as we have edges that are only refereced once
            closeCounter += (i->second.size()-1 - i->second[0]);
        }

    }
    std::cout << "Found " << closeCounter << " non-closed triangles" << std::endl;

    if (closeCounter > 0){
        std::cout << "Error: Model is not closed..." << endl;
        return false;
    }

    std::cout << "Model checked for closeness..." << endl;
    return true;
}




int Mesh::hashEdgeSort(){
    windingError = 0;
    //clean edges are stored in here
    vector<Edge> cleanEdges;
    int key;
    int i, hitcount = 0, counter = 0;

    // remove duplicate edges
    for(i = 0; i < (int) edges.size(); i++){

        //default
        key = 10000000;
        int opposite = 1000000;

        //get the key (smaller vertex index) and assign the bigger vertex index to opposite
        if (edges[i].v[0] < edges[i].v[1]){
            key = edges[i].v[0];
            opposite = edges[i].v[1];
        }
        else if (edges[i].v[0] >= edges[i].v[1]){
            key = edges[i].v[1];
            opposite = edges[i].v[0];
            if (edges[i].v[0] == edges[i].v[1]){
                counter++;
            }
        }

        // key not in map
        if(edgelookup.find(key) == edgelookup.end()) {
            //if we did not find the key, we add a new index inot the hash map
            std::vector<int> temp;
            //init the counter to 0
            temp.push_back(0);
            //push on the bigger vertex
            temp.push_back(opposite);
            //push on default value to ensure equal indexing in the arrays
            companion[key].push_back(0);
            //push on the first vertex, this is used to see later on if the winding is opposite
            companion[key].push_back(edges[i].v[0]);
            //assign data
            edgelookup[key] = temp;
            //push edge
            cleanEdges.push_back(edges[i]);
        }
        else        {
            //if we do find the entry
            //we check through the vector to see if the acomplice vertex has been added
            std::vector<int>::iterator start = edgelookup[key].begin();
            std::vector<int>::iterator end = edgelookup[key].end();
            bool found = false;
            //for counter
            start++;
            int count1 = 1;
            while (start != end){
                //if we do find it, we break as it is already added, thus a duplciate edge
                if (*start == opposite){

                    //CHECKING FOR WINDING TO DETERMINE ORIENTABILITY
                    //we compare which vertex was the first one here, if they v[0]'s are equal that means the winding is equal
                    if (edges[i].v[0] == companion[key][count1]){
                        //have different winding
                        windingError++;
                    }


                    //CHECKING IF THE MODEL IS CLOSED
                    //we increment the counter by 1
                    //see explenation behind method in the closedTest() method
                    edgelookup[key][0] += 1;

                    //CHECKING HOW MANY DUPLCIATE EDGES THERE ARE
                    hitcount++;
                    found = true;
                    break;
                }
                start++;
                count1++;
            }

            //if it is not found, we add a new entry, meaning we have a new destination for a source vertex that already exists
            if (!found){
                //push new bigger key
                edgelookup[key].push_back(opposite);
                //push the edge's winding
                companion[key].push_back(edges[i].v[0]);
                cleanEdges.push_back(edges[i]);
            }
        }
    }

    cerr << "Broken edges found: " << counter << std::endl;
    cerr << "Duplicate edges found = " << hitcount << " of " << (int) edges.size() << endl;
    cerr << "Clean edges = " << (int) cleanEdges.size() << endl;

    no_edges_clean = cleanEdges.size();
    no_edges_dirty = edges.size();

    edges.clear();
    edges = cleanEdges;
    return no_edges_clean;
}





void Mesh::loadFile(std::string filename){
    readSTL(filename);
}
