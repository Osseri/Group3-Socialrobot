#-*- encoding: utf8 -*
from rdflib import *
from rdflib.namespace import OWL, RDF, RDFS
from rdflib.resource import Resource
from rdflib import Graph, Literal, Namespace, URIRef, BNode
from urlparse import urlparse, urljoin, urldefrag, urlsplit
import rospkg

# URI namespace
PDDL = Namespace("http://www.semanticweb.org/pddl#")
SOCIAL = Namespace("http://www.semanticweb.org/socialrobot#")

class ActionReader():

    def __init__(self, rdf_path):
        self.graph = Graph()
        self.rdf_path = rdf_path        
        self.actions={}        
        self.action_list = []
        self.available_action_list = []
        self.requirements = []
        self.constants = []
        self.predicates = {}
        self.predicate_list = []
        self.types = {}
        self.type_list = []
        

        # set main rdf file path
        self.file_format = ".rdf"
        self.rdf_file = self.rdf_path + "action" + self.file_format
        self.readFile()

    def readFile(self):
        try:
            self.graph.parse(self.rdf_file)

            # add imported rdf files into graph instance
            self.graph = self.graph + self.graphOperation(self.graph)
            
            print ("Converting rdf to python dict...")
            if(self.getActionList()):
                self.getRequirements()
                self.getTypes()
                self.getPredicates()
                self.getConstants()

                print("Convert complete.")
            else:
                print("Convert failed.")
        except KeyError as e:
            print("Can't load the rdf file...")
            pass

    def graphOperation(self, graph):        
        imported_graph = self.addImportedGraph(graph)

        return imported_graph

    def addImportedGraph(self, graph):  
        imported_graph = Graph()
        for o in graph.objects(None, OWL.imports):            
            file_name =  str(urlsplit(o).path).replace("/", "")
            print ("Importing " + file_name)
            file_path = self.rdf_path + file_name + self.file_format
            g = Graph()
            g.parse(file_path)
            
            imported_graph = g + self.addImportedGraph(g)

        return imported_graph


    def getRequirements(self):
        if not (None, RDFS.subClassOf, PDDL.requirements) in self.graph:
            print("Graph has not requirements triple")
            return False

        # get the precondition subclass instances
        for r in self.graph.subjects(RDFS.subClassOf, PDDL.requirements):
            self.requirements.append(":"+str(urldefrag(r)[1]))

    def getTypes(self):
        if not (None, RDFS.subClassOf, PDDL.types) in self.graph:
            print("Graph has not types triple")
            return False

        # get the subclass instances
        for url in self.graph.subjects(RDFS.subClassOf, PDDL.types):
            key = str(urldefrag(url)[1])       
            self.types[key] = self.getSubclassOf(url)

    def getSubclassOf(self, url):
        url_dict = {}

        # get the subclass instances
        for sub_url in self.graph.subjects(RDFS.subClassOf, url):
            key = str(urldefrag(sub_url)[1])
            url_dict[key] = self.getSubclassOf(sub_url)
            
        return url_dict

    def getConstants(self):
        pass
    
    def getPredicates(self):
        if not (None, RDFS.subClassOf, PDDL.predicates) in self.graph:
            print("Graph has not predicates triple")
            return False

        # get the subclass instances
        for url in self.graph.subjects(RDFS.subClassOf, PDDL.predicates):
            predicate = str(urldefrag(url)[1])       
            
            template = {'parameters': None}
            self.predicates[predicate] = template
            self.predicate_list.append(predicate)

            params = []
            res = Resource(self.graph, url)

            # get parameter
            if res:
                parameter_num=0
                for i in  list(res.predicates()):
                    if i.qname() == 'pddl:parameter':
                        parameter_num = parameter_num +1               
                for i in range(parameter_num):
                    params.append(None)

                for parameter_res in res.objects(PDDL.parameter):
                    idx, var = self.getParameter(parameter_res)
                    params[int(idx.replace("param", ""))-1] = var
            self.predicates[predicate]['parameter'] = params


    def getAvailableActions(self, hardware_list):
        action_list = []

        if len(self.action_list) == 0:
            return False
        
        for action_name in self.action_list:
            action_res = Resource(self.graph, URIRef(SOCIAL + action_name))
            const_res = action_res.value(SOCIAL.constraints)

            # find hardware_group in constraints resource
            if const_res:
                req_hw = []
                for group in list(const_res.objects(SOCIAL.hardware_group)):
                    req_hw.append(group.qname())
                #check if required hardware are in the hardware_list    
                result =  all(elem in hardware_list  for elem in req_hw)    
                if result:
                    action_list.append(action_name)
                else :
                    pass       
        self.available_action_list = action_list
        return action_list


    def getActionList(self):
        if not (None, RDFS.subClassOf, PDDL.action) in self.graph:
            print("Graph has not action triple")
            return False

        # get the actions's subclass instances
        for s in self.graph.subjects(RDFS.subClassOf, PDDL.action):
            #initialize dict
            action_name =str(urlsplit(s).fragment)
            action_template = {'parameters': None, 'effect': None, 'precondition':None, 'constraints':None, 'primitives':None}
            self.actions[action_name] = action_template
            self.action_list.append(action_name)

            # get parameters
            parameters = self.getActionParameters(URIRef(SOCIAL + action_name))
            self.actions[action_name]['parameters'] = parameters

            # get precondition
            precondition = self.getActionPrecondition(URIRef(SOCIAL + action_name))
            self.actions[action_name]['precondition'] = precondition

            # get effect
            effect = self.getActionEffect(URIRef(SOCIAL + action_name))
            self.actions[action_name]['effect'] = effect

            # get primitive actions
            primitives = self.getPrimitives(URIRef(SOCIAL + action_name))
            self.actions[action_name]['primitives'] = primitives

            # get action constraints
            constraints = self.getConstraints(URIRef(SOCIAL + action_name))
            self.actions[action_name]['constraints'] = constraints

        return True

    def getPrimitives(self, action_uri):
        primitives = []
        action_res = Resource(self.graph, action_uri)
        primitives_res = action_res.value(SOCIAL.primitives)

        # check number of parameters
        if primitives_res:
            primitives_num = len(list(primitives_res.objects()))
            for i in range(primitives_num):
                primitives.append(None)

            for p, o in primitives_res.predicate_objects():
                idx = int(p.qname().replace('sub',''))-1
                primitives[idx] = str(o.qname())

        return primitives
   
    def getConstraints(self, action_uri):
        constraints = {'planner':None, 'controller':None, 'group':None}
        action_res = Resource(self.graph, action_uri)
        constraints_res = action_res.value(SOCIAL.constraints)
        
        # Planner
        planner = []
        if constraints_res.value(SOCIAL.planner) != None:
            planner.append(str(constraints_res.value(SOCIAL.planner).qname().lower()))
            constraints['planner'] = planner 
        # Controller
        controller = []
        if constraints_res.value(SOCIAL.controller) != None:
            controller.append(str(constraints_res.value(SOCIAL.controller).qname().lower()))
            constraints['controller'] = controller

        # Hardware group
        group = []
        if constraints_res.value(SOCIAL.hardware_group) != None:
            group.append(str(constraints_res.value(SOCIAL.hardware_group).qname().lower()))
            constraints['group'] = group

        return constraints


    def getActionPrecondition(self, action_uri):
        precond = []
        action_res = Resource(self.graph, action_uri)
        precond_res = action_res.value(PDDL.precondition)

        if precond_res:
            predicate_num = len(list(precond_res.objects()))

            # for i in range(predicate_num):
            #     precond.append(None)

            for predicate_res in list(precond_res.objects()):
                predicate = self.getActionPredicate(predicate_res)
                precond.append(predicate)

        return precond

    def getActionPredicate(self, predicate_res):
        predicate = []

        # check conditions
        predicate_type = str(list(predicate_res.objects(RDF.type))[0].qname().replace('pddl:',''))

        if predicate_type == 'forall':
            parameter_num = len(list(predicate_res.objects()))
            for i in range(parameter_num):
                predicate.append(None)
                predicate[0] = predicate_type
            param = []
            for parameter_res in predicate_res.objects(PDDL.parameter):
                idx, value = self.getParameter(parameter_res)
                param.append(value)
                predicate[1] = param
            i=2
            for sub_predicate_res in predicate_res.objects(PDDL.predicate):
                sub_predicate = self.getActionPredicate(sub_predicate_res)
                predicate[i] = sub_predicate
                i=i+1
        elif predicate_type == 'when':
            parameter_num = len(list(predicate_res.objects()))
            for i in range(parameter_num):
                predicate.append(None)
                predicate[0] = predicate_type
            # serialize the parameters
            for parameter_res in predicate_res.objects(PDDL.parameter):
                for parameter_predicate_res in parameter_res.objects(PDDL.predicate):
                    parameter_predicate   = self.getActionPredicate(parameter_predicate_res)
                    predicate[1] = parameter_predicate

            # serialize the predicates
            i=2
            for sub_predicate_res in predicate_res.objects(PDDL.predicate):
                sub_predicate = self.getActionPredicate(sub_predicate_res)
                predicate[i] = sub_predicate
                i=i+1

        elif predicate_type == 'not' or predicate_type == 'or' or predicate_type == 'and':
            parameter_num = len(list(predicate_res.objects()))
            for i in range(parameter_num):
                predicate.append(None)
                predicate[0] = predicate_type
            i=1
            for sub_predicate_res in predicate_res.objects(PDDL.predicate):
                sub_predicate = self.getActionPredicate(sub_predicate_res)
                predicate[i] = sub_predicate
                i=i+1  

        else:
            parameter_num = len(list(predicate_res.objects()))
            for i in range(parameter_num):
                predicate.append(None)               
                predicate[0] = predicate_type
            for parameter_res in predicate_res.objects(PDDL.parameter):
                idx, var = self.getParameter(parameter_res)
                predicate[int(idx.replace("param", ""))] = var

        return predicate


    def getActionEffect(self, action_uri):
        effect = []
        action_res = Resource(self.graph, action_uri)
        effect_res = action_res.value(PDDL.effect)

        if effect_res:
            predicate_num = len(list(effect_res.objects()))

            # for i in range(predicate_num):
            #     precond.append(None)

            for predicate_res in list(effect_res.objects()):
                predicate = self.getActionPredicate(predicate_res)
                effect.append(predicate)

        return effect

    def getActionParameters(self, action_url):
        '''
        :param action_url: action name [str]
        :return: paramter dictionary [dict]

        Parameters - parameter
                       |- param1
                       |    |- type, name
                       |- param2
                       |    |- type, name
                       ...
        '''
        params = []
        action_res = Resource(self.graph, action_url)
        parameters_res = action_res.value(PDDL.parameters)

        # check number of parameters
        if parameters_res:
            parameter_num = len(list(parameters_res.objects()))
            for i in range(parameter_num):
                params.append(None)

            for parameter_res in parameters_res.objects(PDDL.parameter):
                idx, var = self.getParameter(parameter_res)
                params[int(idx.replace("param", ""))-1] = var

        return params

    def getParameter(self, parameter_res):
        '''

        :param parameter_res:
        :return:
        '''
        param_dict = {}
        param = ""
        param_type = ""
        param_name = ""
        for p, o in parameter_res.predicate_objects():
            if p.qname() == 'rdf:type':
                param_type = str(o.qname())
            else:
                param = str(p.qname().replace('pddl:',''))
                # check whether the parameter is constant or variable
                if (o.__class__.__name__) == 'Resource':
                    param_name =  str(list(o.objects())[0].qname())
                else:
                    param_name = '?' + str(o.value)
            param_dict['name'] = param_name
            param_dict['type'] = param_type

        return param, param_dict

    def printResource(self, res):
        for s, p, o in  res:
            print (s, p, o)

    def getActionInfo(self, action_name, predicates=[]):
        action_info = {'parameters': None, 'effect': None, 'precondition':None, 'primitives':None, 'controller':None, 'group':None, 'planner':None}

        action_info['parameters'] = self._dict_to_list_param(self.actions[action_name]['parameters'])
        action_info['primitives'] = self.actions[action_name]['primitives']
        action_info['planner'] = self.actions[action_name]['constraints']['planner']
        action_info['group'] = self.actions[action_name]['constraints']['group']
        action_info['controller'] = self.actions[action_name]['constraints']['controller']

        #TODO: from predicates extract effect&precondition list
        if len(predicates)<1:
            action_info['effect'] = []
            action_info['precondition'] = []
        else:
            action_info['effect'] = []
            action_info['precondition'] = []

        return action_info

    def _dict_to_list_param(self, param_dict):
        param_list = []
        for param in param_dict:
            param_name = param['name']
            #remove ? string
            param_name = param_name.replace('?','')
            param_list.append(param_name)
        return param_list


if __name__== "__main__":
    rospack = rospkg.RosPack()
    
    rdf_path = rospack.get_path('socialrobot_actionlib') + "/config/"

    ar = ActionReader(rdf_path)
    #print ar.action_list
    #print ar.available_action_list
    #print ar.actions['hold_object']['constraints']

    # print action info
    # for action_name in ar.action_list:
    #     action = ar.actions[action_name]
    #     print action_name
    #     print ' precondition'
    #     for axiom in action['precondition']:
    #         print '     ', axiom
    #     print ' effect'
    #     for axiom in action['effect']:
    #         print '     ', axiom
    #     print ' parameters'
    #     for axiom in action['parameters']:
    #         print '     ', axiom
    #     print 'primitives'
    #     for axiom in action['primitives']:
    #         print '     ', axiom
    #     print 'controller'
    #     for axiom in action['constraints']['controller']:
    #         print '     ', axiom
    #     print 'group'
    #     for axiom in action['constraints']['group']:
    #         print '     ', axiom
    #     print 'planner'
    #     for axiom in action['constraints']['planner']:
    #         print '     ', axiom
        
    print ar.getActionInfo('hold_object')