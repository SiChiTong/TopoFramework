/*
 * Graph.h
 *
 *  Created on: Jul 12, 2013
 *      Author: sam
 */

#ifndef FRAMEWORK_H_
#define FRAMEWORK_H_

#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <string>

namespace ime
{

// ADT's for building the graph.

/**
 * Every object in the graph is an instance of a Node class. But this node
 * behaves differently according to its responsibilities.
 */
class Node
{
  public:
    typedef std::vector<Node*> Nodes;
    Nodes prevs;
    Nodes nexts;
    unsigned int value;
    bool initialized;
    bool computationNode;

    Node() : value(0), initialized(false), computationNode(false) {}
    virtual ~Node() {}
    virtual unsigned int getValue() const { return value; }
    virtual bool getInitialized()   const { return initialized; }
    virtual const char* getName() const =0;
};

class Module : public ime::Node
{
  public: Module() : ime::Node() {}
  public: virtual ~Module() {}
  public: virtual void init() {}
  public: virtual void execute() {}
};

class Representation: public ime::Node
{
  public: void (*update)(Node* , Node* );
  public: Representation() : ime::Node(), update(0) {}
  public: virtual ~Representation() {}
};

class TopoNode
{
  public:
    virtual ~TopoNode() {}
    virtual void init() =0;
    virtual void update() =0;
    virtual const Node* getNode() const =0;
};

class TopoModule : public ime::TopoNode
{
  public:
    Module* module;
    TopoModule(Module*  module) : module(module) {}
    virtual ~TopoModule() {}
    void init() { module->init(); }
    void update() { module->execute(); }
    const Node* getNode() const { return module; }
};

class TopoRepresentation: public ime::TopoNode
{
  public:
    Module* module;
    Representation* representation;
    TopoRepresentation(Module* module, Representation* representation) : module(module), representation(representation) {}
    virtual ~TopoRepresentation() {}
    void init() {/** For later use */}
    void update() { representation->update(module, representation); }
    const Node* getNode() const { return representation; }
};



class Graph
{
  public:
    class ModuleEntry
    {
      public:
        Node* moduleNode;
        ModuleEntry(Node* moduleNode) : moduleNode(moduleNode) {}
    };

    class RepresentationEntry
    {
      public:
        std::string providedModuleName;
        Node* representationNode;
        void (*update)(Node*, Node*);

        RepresentationEntry(std::string providedModuleName, Node* representationNode, void (*update)(Node*, Node*)) :
            providedModuleName(providedModuleName), representationNode(representationNode), update(update) {}
    };

    class ModuleRepresentationEntry
    {
      public:
        std::string requiredModuleName;
        std::string requiredRepresentationName;

        ModuleRepresentationEntry(std::string requiredModuleName, std::string requiredRepresentationName):
          requiredModuleName(requiredModuleName), requiredRepresentationName(requiredRepresentationName) {}
    };

    typedef std::vector<ModuleEntry*> ModuleVector;
    typedef std::vector<RepresentationEntry*> RepresentationVector;
    typedef std::vector<ModuleRepresentationEntry*> ModuleRepresentationVector;
    typedef std::map<unsigned int, int> InDegreesMap;
    typedef std::map<unsigned int, Node*> GraphStructure;
    ModuleVector moduleVector;
    RepresentationVector representationVector;
    ModuleRepresentationVector moduleRepresentationVector;
    InDegreesMap inDegreesMap;
    GraphStructure graphStructure;
    unsigned int nodeCounter;

    // For topological sort
    typedef std::queue<Node*> TopoQueue;
    typedef std::vector<TopoNode*> GraphOutput;
    TopoQueue topoQueue;
    GraphOutput graphOutput;

    static Graph& getInstance();
    static void deleteInstance();
    void addModule(Node* theInstance);
    void providedRepresentation(const char* moduleName, Node* theInstance, void (*updateRepresentation)(Node* , Node* ));
    void requiredRepresentation(const char* moduleName, const char* representationName);
    Node* getRepresentation(const char* representationName);

    /** Computational resources */
    void computeGraph();
    void topoSort();
    void callGraphOutput();

  protected:
    Graph();
    ~Graph();
    Graph(Graph const&);
    Graph& operator=(Graph const&);

  private:
    static Graph* instance;

  public: /** verbose */
    friend std::ostream& operator<<(std::ostream& out, const ime::Graph& that);
};

std::ostream& operator<<(std::ostream& out, const ime::Graph& that);

// All the computational units are loaded into an instance of this class.
template <class T>
class ModuleLoader
{
  public:
    T* theInstance;
    ModuleLoader() : theInstance(new T) { Graph::getInstance().addModule(theInstance); }
    ~ModuleLoader() { delete theInstance; }
};

template<const char* (*getModuleName)(), void (*updateRepresentation)(ime::Node*, ime::Node*), class T>
class RepresentationProvider
{
  public:
    T* theInstance;
    RepresentationProvider() : theInstance(new T) { Graph::getInstance().providedRepresentation(getModuleName(), theInstance, updateRepresentation); }
    ~RepresentationProvider() { delete theInstance; }
};

template<const char* (*getModuleName)(), const char* (*getRepresentationName)(), class T>
class RepresentationRequierer
{
  public:
    RepresentationRequierer() { Graph::getInstance().requiredRepresentation(getModuleName(), getRepresentationName()); }

  protected:
    T* getRepresentation() const
    {
      static T* theInstance = 0;
      if (theInstance == 0)
        theInstance = (T*) Graph::getInstance().getRepresentation(getRepresentationName());
      return theInstance;
    };
  public:
    const T* operator->() const { return getRepresentation(); }
    const T& operator*()  const { return *(getRepresentation()); }
    operator const T*()   const { return getRepresentation(); }
    const bool isNull()   const { return (getRepresentation() == 0); }

};

template<const char* (*getRepresentationName)(), class T>
class RepresentationUser
{
  public:
    RepresentationUser() {}

  protected:
    T* getRepresentation() const
    {
      static T* theInstance = 0;
      if (theInstance == 0)
        theInstance = (T*) Graph::getInstance().getRepresentation(getRepresentationName());
      return theInstance;
    };
  public:
    const T* operator->() const { return getRepresentation(); }
    const T& operator*()  const { return *(getRepresentation()); }
    operator const T*()   const { return getRepresentation(); }
    const bool isNull()   const { return (getRepresentation() == 0); }

};


// Macros to create the computational units as well as the representations that they provide.
#define MODULE(NAME)                                                        \
class NAME;                                                                 \
class NAME##Base : public ime::Module                                       \
{                                                                           \
  private: typedef NAME##Base _This;                                        \
  public: NAME##Base() : ime::Module() {}                                   \
  public: virtual ~NAME##Base() {}                                          \
  public: const char* getName() const { return #NAME ;}                     \
  public: static const char* getNameStatic() { return #NAME ; }             \

#define REQUIRES(REPRESENTATION)                                            \
private: static const char* getRepresentation##REPRESENTATION() { return #REPRESENTATION ; }       \
protected: ime::RepresentationRequierer<&_This::getNameStatic, &_This::getRepresentation##REPRESENTATION, REPRESENTATION> the##REPRESENTATION;      \

#define PROVIDES(REPRESENTATION)                                            \
protected: virtual void update(REPRESENTATION& the##REPRESENTATION) =0;     \
protected: static void _update##REPRESENTATION(ime::Node* moduleNode, ime::Node* representationNode)         \
          { (((_This*) moduleNode)->update(*((REPRESENTATION*) representationNode))); } \
protected: ime::RepresentationProvider<&_This::getNameStatic, &_This::_update##REPRESENTATION, REPRESENTATION> _the##REPRESENTATION##Provides;      \

#define USES(REPRESENTATION)                                               \
protected: ime::RepresentationUser<&_This::getRepresentation##REPRESENTATION, REPRESENTATION> the##REPRESENTATION;                                  \

#define END_MODULE };                                                      \

#define MAKE_MODULE(NAME) ime::ModuleLoader<NAME> _the##NAME##Module;

#define REPRESENTATION(NAME)                                                \
class NAME;                                                                 \
class NAME##Base : public ime::Representation                               \
{                                                                           \
  private: typedef NAME##Base _This;                                        \
  public: NAME##Base() : ime::Representation() {}                           \
  public: virtual ~NAME##Base() {}                                          \
  public: const char* getName() const { return #NAME ;}                     \
};                                                                          \


}  // namespace ime

#endif /* FRAMEWORK_H_ */
