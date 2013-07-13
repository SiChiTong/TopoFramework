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

class Graph
{
  public:

    // Every object the graph is an instance of a Node. But this node behaves differently according
    // to its responsibilities.
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
        virtual unsigned int getValue() { return value; }
        virtual bool getInitialized() { return initialized; }
        virtual const char* getName() const =0;
    };

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
        void (*updateRepresentation)(Node*, Node*);

        RepresentationEntry(std::string providedModuleName, Node* representationNode, void (*updateRepresentation)(Node*, Node*)) :
            providedModuleName(providedModuleName), representationNode(representationNode), updateRepresentation(updateRepresentation) {}
    };

    class ModuleRepresentationEntry
    {
      public:
        std::string requiredModuleName;
        std::string requiredRepresentationName;

        ModuleRepresentationEntry(std::string requiredModuleName, std::string requiredRepresentationName):
          requiredModuleName(requiredModuleName), requiredRepresentationName(requiredRepresentationName) {}
    };

    typedef std::vector<ModuleEntry*> ModulesVector;
    typedef std::vector<RepresentationEntry*> RepresentationVector;
    typedef std::vector<ModuleRepresentationEntry*> ModuleRepresentationVector;
    typedef std::map<unsigned int, int> InDegreesMap;
    typedef std::map<unsigned int, Node*> GraphStructure;
    ModulesVector modulesVector;
    RepresentationVector representationVector;
    ModuleRepresentationVector moduleRepresentationVector;
    InDegreesMap inDegreesMap;
    GraphStructure graphStructure;
    unsigned int nodeCounter;

    // For topological sort
    typedef std::queue<Node*> TopoQueue;
    typedef std::vector<Node*> GraphOutput;
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

template<const char* (*getModuleName)(), void (*updateRepresentation)(ime::Graph::Node*, ime::Graph::Node*), class T>
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


class Module : public ime::Graph::Node
{
  public: Module() : ime::Graph::Node(){}
  public: virtual ~Module() {}
  public: virtual void init() {}
  public: virtual void execute() {}
};

class Representation: public ime::Graph::Node
{
  public: void (*updateRepresentation)(Node* , Node* );
  public: Representation() : ime::Graph::Node(), updateRepresentation(0) {}
  public: virtual ~Representation() {}
};

// Macros to create the computational units as well as the representations that they provide.
#define MODULE(NAME)                                                        \
class NAME;                                                                 \
class NAME##Base : public ime::Module                                       \
{                                                                           \
  private: typedef NAME##Base _this;                                        \
  public: NAME##Base() : ime::Module() {}                                   \
  public: virtual ~NAME##Base() {}                                          \
  public: const char* getName() const { return #NAME ;}                     \
  public: static const char* getNameStatic() { return #NAME ; }             \

#define REQUIRES(REPRESENTATION)                                            \
private: static const char* getRepresentation##REPRESENTATION() { return #REPRESENTATION ; }       \
protected: ime::RepresentationRequierer<&_this::getNameStatic, &_this::getRepresentation##REPRESENTATION, REPRESENTATION> the##REPRESENTATION;      \

#define PROVIDES(REPRESENTATION)                                            \
protected: virtual void update(REPRESENTATION& the##REPRESENTATION) =0;     \
protected: static void _update##REPRESENTATION(ime::Graph::Node* moduleNode, ime::Graph::Node* representationNode)         \
          { (((_this*) moduleNode)->update(*((REPRESENTATION*) representationNode))); } \
protected: ime::RepresentationProvider<&_this::getNameStatic, &_this::_update##REPRESENTATION, REPRESENTATION> _the##REPRESENTATION##Provides; \

#define USES(REPRESENTATION)                                               \
protected: ime::RepresentationUser<&_this::getRepresentation##REPRESENTATION, REPRESENTATION> the##REPRESENTATION;      \

#define END_MODULE };                                                      \

#define MAKE_MODULE(NAME) ime::ModuleLoader<NAME> _the##NAME##Module;

#define REPRESENTATION(NAME)                                                \
class NAME;                                                                 \
class NAME##Base : public ime::Representation                               \
{                                                                           \
  private: typedef NAME##Base _this;                                        \
  public: NAME##Base() : ime::Representation() {}                           \
  public: virtual ~NAME##Base() {}                                          \
  public: const char* getName() const { return #NAME ;}                     \
};                                                                          \


}  // namespace ime

#endif /* FRAMEWORK_H_ */
