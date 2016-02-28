#if HAVE_CONFIG_H
# include <config.h>
#endif

#include <test/testutil.h>
#include "test_mesh.h"
#include <stdio.h>
#include <cstdint>
#include <sstream>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/extensions/HelperMacros.h>

void TestMesh::setUp()
{
    mesh = new Mesh();
}

void TestMesh::tearDown()
{
    delete mesh;
}

void TestMesh::testMeshing()
{
    //edit file path for correct access to stl files -> should be fine
    std::string header = "./meshes/";


    //TEST 1A
    //TESTING TO SEE IF THE UNIQUE EDGE COUNT WORKS WITH THE BUNNY FILE
    Mesh test1a;
    CPPUNIT_ASSERT(test1a.readSTL(header+"bunny.stl"));
    CPPUNIT_ASSERT(test1a.prepareEdges() == 104288);


    //TEST 1B
    //TESTING TO SEE IF THE UNIQUE EDGE COUNT WORKS WITH THE DRAGON FILE
    Mesh test1b;
    CPPUNIT_ASSERT(test1b.readSTL(header+"dragon.stl"));
    CPPUNIT_ASSERT(test1b.prepareEdges() == 896925);


    //TEST 1C
    //BASIC PRE-EXISTING TEST FOR BUNNY
    CPPUNIT_ASSERT(mesh->readSTL(header+"bunny.stl"));
    CPPUNIT_ASSERT(mesh->basicValidity());
    CPPUNIT_ASSERT(!mesh->manifoldValidity()); // bunny has known holes in the bottom


    //TEST 2
    //TESTING IF EACH METHOD IN THE BASIC VALIDITY & 2-MANIFOLD VALIDITY WORKS USING A TRIANGULAR PYRAMID
    if (true){
        Mesh pyramid;
        CPPUNIT_ASSERT(pyramid.readSTL(header+"pyramid.stl"));
        pyramid.prepareEdges();
        CPPUNIT_ASSERT(pyramid.checkEulerCharBool());
        CPPUNIT_ASSERT(pyramid.danglingVerticeCheck());
        CPPUNIT_ASSERT(pyramid.vertexBoundsTest());
        CPPUNIT_ASSERT(pyramid.closedTest());
        CPPUNIT_ASSERT(pyramid.orientableTest());
        CPPUNIT_ASSERT(pyramid.manifoldTest());
    }

    //TEST 3A
    //TESTING IF EACH METHOD IN THE BASIC VALIDITY & 2-MANIFOLD VALIDITY WORKS USING A CUBE
    if (true){
        Mesh cube;
        CPPUNIT_ASSERT(cube.readSTL(header+"Cube2.stl"));
        cube.prepareEdges();
        CPPUNIT_ASSERT(cube.checkEulerCharBool());
        CPPUNIT_ASSERT(cube.danglingVerticeCheck());
        CPPUNIT_ASSERT(cube.vertexBoundsTest());
        CPPUNIT_ASSERT(cube.closedTest());
        CPPUNIT_ASSERT(cube.orientableTest());
        CPPUNIT_ASSERT(cube.manifoldTest());
    }



    //TEST 4
    //TESTING EULER METHOD - ASSERT TRUE CASE
    if (true){
        Mesh test4;
        CPPUNIT_ASSERT(test4.readSTL(header+"Cube2.stl"));
        test4.prepareEdges();
        CPPUNIT_ASSERT(test4.checkEulerCharInt() == 2);
    }

    //TEST 5
    //TESTING EULER METHOD - ASSERT TRUE CASE 2
    if (true){
        Mesh test5;
        CPPUNIT_ASSERT(test5.readSTL(header+"TestGenus1.stl"));
        test5.prepareEdges();
        CPPUNIT_ASSERT(test5.checkEulerCharInt() == 0);
    }

    //TEST 6
    //TEST DANGLING VERTICES METHOD - ASSERT TRUE CASE
    if (true){
        Mesh test6;
        CPPUNIT_ASSERT(test6.readSTL(header+"Cube2.stl"));
        test6.prepareEdges();
        CPPUNIT_ASSERT(test6.danglingVerticeCheck());
    }

    //TEST 7
    //TEST DANGLING VERTICES METHOD - ASSERT FALSE CASE
    if (true){
        Mesh test7;
        CPPUNIT_ASSERT(test7.readSTL(header+"TestDanglingVertices.stl"));
        test7.prepareEdges();
        //CPPUNIT_ASSERT(!test7.danglingVerticeCheck());
        //TODO test fails, as a I can't get a model with dangling vertices, blender seems to cull them on export or stl read in culls thems
    }

    //TEST 8
    //TESTING VERTEX OUT OF BOUNDS METHOD - ASSERT TRUE CASE
    if (true){
        Mesh test8;
        CPPUNIT_ASSERT(test8.readSTL(header+"TestGenus1.stl"));
        test8.prepareEdges();
        CPPUNIT_ASSERT(test8.vertexBoundsTest());
        //TODO how to prove positive case
    }

    //TEST 9
    //TESTING VERTEX OUT OF BOUNDS METHOD - ASSERT FALSE CASE
    if (true){
        Mesh test9;
        CPPUNIT_ASSERT(test9.readSTL(header+"TestGenus1.stl"));
        test9.prepareEdges();
        //TODO how to test negative case?
    }

    //TEST 10
    //TESTING CLOSED MODEL METHOD - ASSERT TRUE CASE
    if (true){
        Mesh test10;
        CPPUNIT_ASSERT(test10.readSTL(header+"basic_sphere.stl"));
        CPPUNIT_ASSERT(test10.basicValidity());
        CPPUNIT_ASSERT(test10.closedTest());
    }


    //TEST 11
    //TESTING CLOSED MODEL METHOD - ASSERT FALSE CASE
    if (true){
        Mesh test11;
        CPPUNIT_ASSERT(test11.readSTL(header+"TestClosed.stl"));
        CPPUNIT_ASSERT(test11.basicValidity());
        CPPUNIT_ASSERT(!test11.closedTest());
    }

    //TEST 12
    //TESTING ORIENTABLE METHOD - ASSERT TRUE CASE
    if (true){
        Mesh test12;
        CPPUNIT_ASSERT(test12.readSTL(header+"basic_sphere.stl"));
        CPPUNIT_ASSERT(test12.basicValidity());
        CPPUNIT_ASSERT(test12.closedTest());
        CPPUNIT_ASSERT(test12.orientableTest());
    }

    //TEST 13
    //TESTING ORIENTABLE METHOD - ASSERT FALSE CASE
    if (true){
        Mesh test13;
        CPPUNIT_ASSERT(test13.readSTL(header+"TestOrientable.stl"));
        CPPUNIT_ASSERT(test13.basicValidity());
        CPPUNIT_ASSERT(test13.closedTest());
        CPPUNIT_ASSERT(!test13.orientableTest());
    }

    //TEST 14
    //TESTING 2-MANIFOLD METHOD - ASSERT TRUE CASE
    if (true){
        Mesh test14;
        CPPUNIT_ASSERT(test14.readSTL(header+"basic_sphere.stl"));
        CPPUNIT_ASSERT(test14.basicValidity());
        CPPUNIT_ASSERT(test14.closedTest());
        CPPUNIT_ASSERT(test14.orientableTest());
        CPPUNIT_ASSERT(test14.manifoldTest());
    }

    //TEST 15
    //TESTING 2-MANIFOLD METHOD - ASSERT FALSE CASE
    if (true){
        Mesh test15;
        CPPUNIT_ASSERT(test15.readSTL(header+"TwoManifoldTestFail.stl"));
        CPPUNIT_ASSERT(test15.basicValidity());
        CPPUNIT_ASSERT(test15.closedTest());
        //CPPUNIT_ASSERT(test15.orientableTest());
        CPPUNIT_ASSERT(!test15.manifoldTest());
        //TODO need to find model that passed closed and orientable? --> method fails orientable thus uncommented
    }




}

//#if 0 /* Disabled since it crashes the whole test suite */
CPPUNIT_TEST_SUITE_NAMED_REGISTRATION(TestMesh, TestSet::perCommit());
//#endif
