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
  for (ime::Graph::ModuleRepresentationVector::iterator iter = moduleRepresentationVector.begin();
      iter != moduleRepresentationVector.end(); ++iter)
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

  theInstance->value = nodeCounter++;
  theInstance->computationNode = true;
  ime::Graph::ModuleEntry* newModuleEntry = new ime::Graph::ModuleEntry(theInstance);
  moduleVector.push_back(newModuleEntry);
  inDegreesMap.insert(std::make_pair(theInstance->value, 0));
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
  theInstance->value = nodeCounter++;
  ime::Graph::RepresentationEntry* newRepresentationEntry = new ime::Graph::RepresentationEntry(
      std::string(moduleName), theInstance, updateRepresentation);
  representationVector.push_back(newRepresentationEntry);
  inDegreesMap.insert(std::make_pair(theInstance->value, 0));
}

void ime::Graph::requiredRepresentation(const char* moduleName, const char* representationName)
{
  ime::Graph::ModuleRepresentationEntry* newModuleRepresentationEntry =
      new ime::Graph::ModuleRepresentationEntry(std::string(moduleName),
          std::string(representationName));
  moduleRepresentationVector.push_back(newModuleRepresentationEntry);
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
        std::make_pair(moduleEntry->moduleNode->getValue(), moduleEntry->moduleNode));
  }

  // 2) Provides representations
  for (ime::Graph::RepresentationVector::iterator iter = representationVector.begin();
      iter != representationVector.end(); ++iter)
  {
    ime::Graph::RepresentationEntry* representationEntry = *iter;
    graphStructure.insert(
        std::make_pair(representationEntry->representationNode->getValue(),
            representationEntry->representationNode));

    for (ime::Graph::ModuleVector::iterator iter2 = moduleVector.begin();
        iter2 != moduleVector.end(); ++iter2)
    {
      if (std::string((*iter2)->moduleNode->getName()).compare(
          representationEntry->providedModuleName) == 0)
      {
        representationEntry->representationNode->prevs.push_back((*iter2)->moduleNode);
        ((ime::Representation*) representationEntry->representationNode)->update =
            representationEntry->update;
        (*iter2)->moduleNode->nexts.push_back(representationEntry->representationNode);
      }
    }
  }

  // 3) Requires representations
  for (ime::Graph::ModuleRepresentationVector::iterator iter = moduleRepresentationVector.begin();
      iter != moduleRepresentationVector.end(); ++iter)
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

    representationNode->nexts.push_back(moduleNode);

  }
}

void ime::Graph::topoSort()
{
  // Calculate in-degrees
  for (ime::Graph::GraphStructure::iterator i = graphStructure.begin(); i != graphStructure.end();
      ++i)
  {
    Node* x = i->second;
    for (ime::Node::Nodes::iterator j = x->nexts.begin(); j != x->nexts.end(); ++j)
    {
      ++inDegreesMap[(*j)->value];
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
    if (x->computationNode)
      topoNode = new TopoModule((Module*) x);
    else
      topoNode = new TopoRepresentation(((Module*) *(x->prevs.begin())), (Representation*) x);

    if (x->initialized)
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
    x->initialized = true;
    graphOutput.push_back(topoNode);
    for (ime::Node::Nodes::iterator j = x->nexts.begin(); j != x->nexts.end(); ++j)
    {
      ime::Node* y = *j;
      --inDegreesMap[y->getValue()];
      if (inDegreesMap[y->getValue()] == 0)
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

void ime::Graph::callGraphOutput()
{
  // 1) Initialize
  for (ime::Graph::GraphOutput::iterator iter = graphOutput.begin(); iter != graphOutput.end();
      ++iter)
  {
    // 1) Init()
    (*iter)->init();
  }

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
    out << moduleEntry->moduleNode->getName() << " " << moduleEntry->moduleNode->getValue()
        << std::endl;
  }

  out << std::endl;
  for (ime::Graph::RepresentationVector::const_iterator iter = that.representationVector.begin();
      iter != that.representationVector.end(); ++iter)
  {
    const ime::Graph::RepresentationEntry* representationEntry = *iter;
    out << representationEntry->representationNode->getName() << " "
        << representationEntry->representationNode->getValue() << " "
        << representationEntry->providedModuleName << std::endl;
  }

  out << std::endl;

  for (ime::Graph::GraphStructure::const_iterator iter = that.graphStructure.begin();
      iter != that.graphStructure.end(); ++iter)
  {
    out << "[" << iter->first << ":" << iter->second->getName() << "] ";
    for (ime::Node::Nodes::const_iterator iter2 = iter->second->nexts.begin();
        iter2 != iter->second->nexts.end(); ++iter2)
    {
      ime::Node* next = *iter2;
      out << "[" << next->getValue() << ":" << next->getName() << "] ";
    }
    out << std::endl;
  }

  for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
      iter != that.graphOutput.end(); ++iter)
  {
    const ime::Node* x = (*iter)->getNode();
    out << x->getValue() << ":" << x->getName() << std::endl;
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
      if (x->computationNode)
        graph << " " << x->getName() << "; ";
    }
    graph << "\n";
    graph << "\t node [shape=ellipse, color=lightpink, style=filled]; ";
    for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
        iter != that.graphOutput.end(); ++iter)
    {
      const ime::Node* x = (*iter)->getNode();
      if (!x->computationNode)
        graph << " " << x->getName() << "; ";
    }
    graph << "\n";
    for (ime::Graph::GraphOutput::const_iterator iter = that.graphOutput.begin();
        iter != that.graphOutput.end(); ++iter)
    {
      const ime::Node* x = (*iter)->getNode();
      if (!x->nexts.empty())
      {
        for (ime::Node::Nodes::const_iterator j = x->nexts.begin(); j != x->nexts.end(); ++j)
        {
          ime::Node* y = *j;
          graph << "\t" << x->getName() << " -> " << y->getName() << "; \n";
        }
      }
      else
      {
        graph << "\t" << x->getName() << "; \n";
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

