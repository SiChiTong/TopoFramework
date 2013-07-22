/*
 * Graph.cpp
 *
 *  Created on: Jul 12, 2013
 *      Author: sam
 */

#include "Framework.h"
#include <cassert>
#include <cstring>
#include <string>
#include <fstream>
#include <istream>
#include <iterator>
#include <algorithm>
#include <limits>

ime::Graph* ime::Graph::instance = 0;

ime::Graph::Graph() :
    nodeCounter(0)
{
}

ime::Graph::~Graph()
{
  for (ime::Graph::ModuleVector::iterator iter = moduleVector.begin(); iter != moduleVector.end();
      ++iter)
    delete *iter;
  for (ime::Graph::RepresentationVector::iterator iter = representationVector.begin();
      iter != representationVector.end(); ++iter)
    delete *iter;
  for (ime::Graph::ModuleRepresentationVector::iterator iter =
      moduleRepresentationRequiredVector.begin(); iter != moduleRepresentationRequiredVector.end();
      ++iter)
    delete *iter;
  for (ime::Graph::ModuleRepresentationVector::iterator iter =
      moduleRepresentationUsedVector.begin(); iter != moduleRepresentationUsedVector.end(); ++iter)
    delete *iter;
  for (ime::Graph::GraphOutput::iterator iter = graphOutput.begin(); iter != graphOutput.end();
      ++iter)
    delete *iter;
}

ime::Graph& ime::Graph::getInstance()
{
  if (!instance)
    instance = new Graph;
  return *instance;
}

void ime::Graph::deleteInstance()
{
  if (instance)
    delete instance;
}

void ime::Graph::addModule(ime::Node* theInstance)
{
  // Check if a module type exits
  for (ime::Graph::ModuleVector::const_iterator iter = moduleVector.begin();
      iter != moduleVector.end(); ++iter)
  {
    if (std::string((*iter)->moduleNode->getName()).compare(theInstance->getName()) == 0)
    {
      std::cout << "ERROR! moduleByName=" << theInstance->getName() << " exists!" << std::endl;
      exit(EXIT_FAILURE);
    }
  }

  theInstance->setIndex(nodeCounter++);
  theInstance->setComputationNode(true);
  ime::Graph::ModuleEntry* newModuleEntry = new ime::Graph::ModuleEntry(theInstance);
  moduleVector.push_back(newModuleEntry);
  inDegreesMap.insert(std::make_pair(theInstance->getIndex(), 0));
}

void ime::Graph::providedRepresentation(const char* moduleName, ime::Node* theInstance,
    void (*updateRepresentation)(Node*, Node*))
{
  // Check if a representation type exits
  for (ime::Graph::RepresentationVector::const_iterator iter = representationVector.begin();
      iter != representationVector.end(); ++iter)
  {
    if (std::string((*iter)->representationNode->getName()).compare(theInstance->getName()) == 0)
    {
      std::cout << "ERROR! representationByName=" << theInstance->getName() << " exists!"
          << std::endl;
      exit(EXIT_FAILURE);
    }
  }
  theInstance->setIndex(nodeCounter++);
  ime::Graph::RepresentationEntry* newRepresentationEntry = new ime::Graph::RepresentationEntry(
      std::string(moduleName), theInstance, updateRepresentation);
  representationVector.push_back(newRepresentationEntry);
  inDegreesMap.insert(std::make_pair(theInstance->getIndex(), 0));
}

void ime::Graph::requiredRepresentation(const char* moduleName, const char* representationName)
{
  ime::Graph::ModuleRepresentationEntry* newModuleRepresentationEntry =
      new ime::Graph::ModuleRepresentationEntry(std::string(moduleName),
          std::string(representationName));
  moduleRepresentationRequiredVector.push_back(newModuleRepresentationEntry);
}

void ime::Graph::usedRepresentation(const char* moduleName, const char* representationName)
{
  ime::Graph::ModuleRepresentationEntry* newModuleRepresentationEntry =
      new ime::Graph::ModuleRepresentationEntry(std::string(moduleName),
          std::string(representationName));
  moduleRepresentationUsedVector.push_back(newModuleRepresentationEntry);
}

ime::Node* ime::Graph::getRepresentation(const char* representationName)
{
  for (ime::Graph::RepresentationVector::iterator iter = representationVector.begin();
      iter != representationVector.end(); ++iter)
  {
    ime::Graph::RepresentationEntry* representationEntry = *iter;
    if (std::string(representationEntry->representationNode->getName()).compare(representationName)
        == 0)
    {
      assert(representationEntry->representationNode->getInitialized());
      return representationEntry->representationNode;
    }
  }
  /** This is a double check and nothing should enter at this point */
  assert(false);
  return 0;
}

void ime::Graph::computeGraph()
{
  // 1) Add modules
  for (ime::Graph::ModuleVector::iterator iter = moduleVector.begin(); iter != moduleVector.end();
      ++iter)
  {
    ime::Graph::ModuleEntry* moduleEntry = *iter;
    graphStructure.insert(
        std::make_pair(moduleEntry->moduleNode->getIndex(), moduleEntry->moduleNode));
  }

  // 2) Provides representations
  for (ime::Graph::RepresentationVector::iterator iter = representationVector.begin();
      iter != representationVector.end(); ++iter)
  {
    ime::Graph::RepresentationEntry* representationEntry = *iter;
    graphStructure.insert(
        std::make_pair(representationEntry->representationNode->getIndex(),
            representationEntry->representationNode));

    for (ime::Graph::ModuleVector::iterator iter2 = moduleVector.begin();
        iter2 != moduleVector.end(); ++iter2)
    {
      if (std::string((*iter2)->moduleNode->getName()).compare(
          representationEntry->providedModuleName) == 0)
      {
        representationEntry->representationNode->addPreviousNode((*iter2)->moduleNode);
        ((ime::Representation*) representationEntry->representationNode)->updateThis =
            representationEntry->update;
        (*iter2)->moduleNode->addNextNode(representationEntry->representationNode);
      }
    }
  }

  // 3) Requires representations
  for (ime::Graph::ModuleRepresentationVector::iterator iter =
      moduleRepresentationRequiredVector.begin(); iter != moduleRepresentationRequiredVector.end();
      ++iter)
  {
    ime::Graph::ModuleRepresentationEntry* moduleRepresentationEntry = *iter;

    ime::Node *moduleNode = 0, *representationNode = 0;
    for (ime::Graph::ModuleVector::iterator iter2 = moduleVector.begin();
        iter2 != moduleVector.end(); ++iter2)
    {
      if (std::string((*iter2)->moduleNode->getName()).compare(
          moduleRepresentationEntry->requiredModuleName) == 0)
      {
        moduleNode = (*iter2)->moduleNode;
      }
    }

    for (ime::Graph::RepresentationVector::iterator iter2 = representationVector.begin();
        iter2 != representationVector.end(); ++iter2)
    {
      if (std::string((*iter2)->representationNode->getName()).compare(
          moduleRepresentationEntry->requiredRepresentationName) == 0)
      {
        representationNode = (*iter2)->representationNode;
      }
    }

    if (moduleNode == 0)
    {
      std::cout << "requiredModuleName=" << moduleRepresentationEntry->requiredModuleName
          << " is missing!" << std::endl;
    }
    if (representationNode == 0)
    {
      std::cout << "requiredRepresentationName="
          << moduleRepresentationEntry->requiredRepresentationName << " is missing!" << std::endl;
    }
    assert(moduleNode && representationNode);
    representationNode->addNextNode(moduleNode);

  }

  // 4) Uses representation
  for (ime::Graph::ModuleRepresentationVector::iterator iter =
      moduleRepresentationUsedVector.begin(); iter != moduleRepresentationUsedVector.end(); ++iter)
  {
    ime::Graph::ModuleRepresentationEntry* moduleRepresentationEntry = *iter;

    ime::Node *moduleNode = 0, *representationNode = 0;
    for (ime::Graph::ModuleVector::iterator iter2 = moduleVector.begin();
        iter2 != moduleVector.end(); ++iter2)
    {
      if (std::string((*iter2)->moduleNode->getName()).compare(
          moduleRepresentationEntry->requiredModuleName) == 0)
      {
        moduleNode = (*iter2)->moduleNode;
      }
    }

    for (ime::Graph::RepresentationVector::iterator iter2 = representationVector.begin();
        iter2 != representationVector.end(); ++iter2)
    {
      if (std::string((*iter2)->representationNode->getName()).compare(
          moduleRepresentationEntry->requiredRepresentationName) == 0)
      {
        representationNode = (*iter2)->representationNode;
      }
    }

    assert(moduleNode && representationNode);

    representationNode->addAuxiliaryNode(moduleNode);

  }

}

void ime::Graph::topoSort()
{
  // Calculate in-degrees
  for (ime::Graph::GraphStructure::iterator i = graphStructure.begin(); i != graphStructure.end();
      ++i)
  {
    Node* x = i->second;
    for (ime::Node::iterator j = x->nextNodesBegin(); j != x->nextNodesEnd(); ++j)
    {
      ++inDegreesMap[(*j)->getIndex()];
    }
  }

  // Initialize the loop
  for (ime::Graph::InDegreesMap::iterator i = inDegreesMap.begin(); i != inDegreesMap.end(); ++i)
  {
    if (i->second == 0)
    {
      Node* x = graphStructure[i->first];
      topoQueue.push(x);
    }
  }

  // Main loop
  while (!topoQueue.empty())
  {
    Node* x = topoQueue.front();
    topoQueue.pop();

    TopoNode* topoNode = 0;
    if (x->getComputationNode())
      topoNode = new TopoModule((Module*) x);
    else
      topoNode = new TopoRepresentation(((Module*) *(x->previousNodesBegin())),
          (Representation*) x);

    if (x->getInitialized())
    {
      graphOutput.push_back(topoNode);
      std::cout << "ERROR! Cycle detected!" << std::endl;
      int tabCounter = 0;
      for (ime::Graph::GraphOutput::const_iterator j = graphOutput.begin(); j != graphOutput.end();
          ++j)
      {
        for (int k = 0; k < tabCounter; k++)
          std::cout << "\t";
        const Node* y = (*j)->getNode();
        std::cout << y->getName() << std::endl;
        ++tabCounter;
      }
      exit(EXIT_FAILURE);
    }
    x->setInitialized(true);
    graphOutput.push_back(topoNode);
    for (ime::Node::iterator j = x->nextNodesBegin(); j != x->nextNodesEnd(); ++j)
    {
      ime::Node* y = *j;
      --inDegreesMap[y->getIndex()];
      if (inDegreesMap[y->getIndex()] == 0)
      {
        topoQueue.push(y);
      }
    }
  }

  if (graphOutput.size() != graphStructure.size())
  {
    std::cout << "ERROR! cycle detected!" << std::endl;
    exit(EXIT_FAILURE);
  }
  assert(graphOutput.size() == graphStructure.size());
}

void ime::Graph::graphOutputAllocate()
{
  // 1) Allocate
  for (ime::Graph::GraphOutput::iterator iter = graphOutput.begin(); iter != graphOutput.end();
      ++iter)
  {
    // 1) Init()
    (*iter)->allocate();
  }
}

void ime::Graph::graphOutputRelease()
{
  // 3) Release
  for (ime::Graph::GraphOutput::iterator iter = graphOutput.begin(); iter != graphOutput.end();
      ++iter)
  {
    // 3) Release()
    (*iter)->release();
  }
}

void ime::Graph::graphOutputUpdate()
{
  // 2) Execute / Update
  for (ime::Graph::GraphOutput::iterator iter = graphOutput.begin(); iter != graphOutput.end();
      ++iter)
  {
    // 2.1) Execute() / 2.2) Update()
    (*iter)->update();
  }
}

std::ostream& ime::operator<<(std::ostream& out, const ime::Graph& that)
{
  out << std::endl << std::endl;
  // This shows the raw graph
  for (ime::Graph::ModuleVector::const_iterator iter = that.moduleVector.begin();
      iter != that.moduleVector.end(); ++iter)
  {
    const ime::Graph::ModuleEntry* moduleEntry = *iter;
    out << moduleEntry->moduleNode->getName() << " " << moduleEntry->moduleNode->getIndex()
        << std::endl;
  }

  out << std::endl;
  for (ime::Graph::RepresentationVector::const_iterator iter = that.representationVector.begin();
      iter != that.representationVector.end(); ++iter)
  {
    const ime::Graph::RepresentationEntry* representationEntry = *iter;
    out << representationEntry->representationNode->getName() << " "
        << representationEntry->representationNode->getIndex() << " "
        << representationEntry->providedModuleName << std::endl;
  }

  out << std::endl;

  for (ime::Graph::GraphStructure::const_iterator iter = that.graphStructure.begin();
      iter != that.graphStructure.end(); ++iter)
  {
    out << "[" << iter->first << ":" << iter->second->getName() << "] ";
    for (ime::Node::const_iterator iter2 = iter->second->nextNodesBegin();
        iter2 != iter->second->nextNodesEnd(); ++iter2)
    {
      ime::Node* next = *iter2;
      out << "[" << next->getIndex() << ":" << next->getName() << "] ";
    }
    out << std::endl;
  }

  for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
      iter != that.graphOutput.end(); ++iter)
  {
    const ime::Node* x = (*iter)->getNode();
    out << x->getIndex() << ":" << x->getName() << std::endl;
  }

  // Graphviz output
  out << std::endl << std::endl;
  out.flush();
  std::ofstream graph("graph_structure.dot");
  if (graph.is_open())
  {
    graph << "digraph G {\n";
    graph << "\t node [shape=box, color=lightblue2, style=filled]; ";
    for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
        iter != that.graphOutput.end(); ++iter)
    {
      const ime::Node* x = (*iter)->getNode();
      if (x->getComputationNode())
        graph << " " << x->getName() << "; ";
    }
    graph << "\n";
    graph << "\t node [shape=ellipse, color=lightpink, style=filled]; ";
    for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
        iter != that.graphOutput.end(); ++iter)
    {
      const ime::Node* x = (*iter)->getNode();
      if (!x->getComputationNode())
        graph << " " << x->getName() << "; ";
    }
    graph << "\n";
    for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
        iter != that.graphOutput.end(); ++iter)
    {
      const ime::Node* x = (*iter)->getNode();
      if (!x->nextNodesEmpty())
      {
        for (ime::Node::const_iterator j = x->nextNodesBegin(); j != x->nextNodesEnd(); ++j)
        {
          ime::Node* y = *j;
          if (y->getComputationNode())
            graph << "edge [color=green]; \n";
          else
            graph << "edge [color=blue]; \n";
          graph << "\t" << x->getName() << " -> " << y->getName() << "; \n";
        }
      }
      else
      {
        graph << "\t" << x->getName() << "; \n";
      }
    }
    graph << "edge [color=red]; \n";
    for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
        iter != that.graphOutput.end(); ++iter)
    {
      const ime::Node* x = (*iter)->getNode();
      if (!x->auxiliaryNodesEmpty())
      {
        for (ime::Node::const_iterator j = x->auxiliaryNodesBegin(); j != x->auxiliaryNodesEnd();
            ++j)
        {
          ime::Node* y = *j;
          graph << "\t" << x->getName() << " -> " << y->getName() << "; \n";
        }
      }
    }

    graph << "\t fontsize=20; \n";
    graph << "} \n";
    graph.close();
  }
  else
  {
    std::cerr << "ERROR! unable to open the graph_structure.dot file" << std::endl;
  }
  return out;
}

