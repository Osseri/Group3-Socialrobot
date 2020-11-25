#!/usr/bin/env python
import re
from action import Action

class PDDL_Parser:

    SUPPORTED_REQUIREMENTS = [':requirements',
                            ':strips',
                            ':adl',
                            ':disjunctive-preconditions',
                            ':negative-preconditions',
                            ':typing',
                            ':equality']
    action = ''

    # ------------------------------------------
    # Tokens
    # ------------------------------------------

    def scan_tokens(self, filename):
        with open(filename,'r') as f:
            # Remove single line comments
            str = re.sub(r';.*$', '', f.read(), flags=re.MULTILINE).lower()
        # Tokenize
        stack = []
        list = []
        for t in re.findall(r'[()]|[^\s()]+', str):
            if t == '(':
                stack.append(list)
                list = []
            elif t == ')':
                if stack:
                    l = list
                    list = stack.pop()
                    list.append(l)
                else:
                    raise Exception('Missing open parentheses')
            else:
                list.append(t)
        if stack:
            raise Exception('Missing close parentheses')
        if len(list) != 1:
            raise Exception('Malformed expression')
        return list[0]

    #-----------------------------------------------
    # Parse domain
    #-----------------------------------------------

    def parse_domain(self, domain_filename):
        tokens = self.scan_tokens(domain_filename)
        if type(tokens) is list and tokens.pop(0) == 'define':
            self.domain_name = 'unknown'
            self.requirements = []
            self.types = []
            self.constants = []
            self.actions = {}
            self.predicates = {}
            while tokens:
                group = tokens.pop(0)
                t = group.pop(0)
                if   t == 'domain':
                    self.domain_name = group[0]
                elif t == ':requirements':
                    for req in group:
                        if not req in self.SUPPORTED_REQUIREMENTS:
                            raise Exception('Requirement ' + req + ' not supported')
                    self.requirements = group
                elif t == ':predicates':
                    self.parse_predicates(group)
                elif t == ':types':
                    self.types = group
                elif t == ':constants':
                    self.parse_constants(group)
                elif t == ':action':
                    self.parse_action(group)
                else: print(str(t) + ' is not recognized in domain')
        else:
            raise Exception('File ' + domain_filename + ' does not match domain pattern')

    #-----------------------------------------------
    # Parse predicates
    #-----------------------------------------------

    def parse_predicates(self, group):
        for pred in group:
            predicate_name = pred.pop(0)
            if predicate_name in self.predicates.keys():
                raise Exception('Predicate ' + predicate_name + ' redefined')
            self.predicates[predicate_name] = self.split_parameters(pred)

    #-----------------------------------------------
    # Parse constants
    #-----------------------------------------------

    def parse_constants(self, group):
        if not type(group) is list:
            raise Exception('Error with contraints')
        constants = []
        untyped_constants = []
        while group:
            t = group.pop(0)
            if t == '-':
                if not untyped_constants:
                    raise Exception('Unexpected hyphen in contraints')
                ptype = group.pop(0)
                while untyped_constants:
                    constants.append([untyped_constants.pop(0), ptype])
            else:
                untyped_constants.append(t)

        const_dict = {}
        for const in constants:
            const_dict[const[1]] = []
        for const in constants:
            const_dict[const[1]].append(const[0])

        self.constants = const_dict

    #-----------------------------------------------
    # Parse action
    #-----------------------------------------------

    def parse_action(self, group):
        name = group.pop(0)
        self.action = name
        self.actions[name] = {}
        if not type(name) is str:
            raise Exception('Action without name definition')

        action = {}
        planner = []
        controller = []
        hardware_group = []               
        primitives = []

        while group:
            t = group.pop(0)
            if t == ':parameters':
                self.actions[name]['parameters'] = self.split_parameters(group.pop(0))
            elif t == ':precondition':
                p = group.pop(0)
                self.actions[name]['precondition'] = self.parse_preconditions(p)
            elif t == ':effect':
                p = group.pop(0)
                self.actions[name]['effect'] = self.parse_effects(p)
            elif t == ':constraints':
                self.split_contraints(group.pop(0), planner, controller, hardware_group)
                self.actions[name]['planner'] = planner
                self.actions[name]['controller'] = controller
                self.actions[name]['group'] = hardware_group
            elif t == ':primitives': 
                self.split_primitives(group.pop(0), primitives)
                self.actions[name]['primitives'] = primitives
            else: print(str(t) + ' is not recognized in action')


    #-----------------------------------------------
    # Parse preconditions
    #-----------------------------------------------
    def parse_preconditions(self, group):
        predicates = []
        #remove first 'and'
        pred_list = group[1:]
        for pred in pred_list:
            self.split_predicates(pred, predicates)
        return predicates

    #-----------------------------------------------
    # Parse effects
    #-----------------------------------------------
    def parse_effects(self, group):
        predicates = []
        #remove first 'and'
        pred_list = group[1:]
        for pred in pred_list:
            self.split_predicates(pred, predicates)
        return predicates 
        
    #-----------------------------------------------
    # Split contraints
    #-----------------------------------------------
    def split_contraints(self, group, planner, controller, hardware_group):
        if not type(group) is list:
            raise Exception('Error with contraints')
        constraints = []
        untyped_constraints = []
        while group:
            t = group.pop(0)
            if t == '-':
                if not untyped_constraints:
                    raise Exception('Unexpected hyphen in contraints')
                ptype = group.pop(0)
                while untyped_constraints:
                    constraints.append([untyped_constraints.pop(0), ptype])
            else:
                untyped_constraints.append(t)

        for const in constraints:
            const_type = const[-1]
            if const_type == 'controller':
                controller.append(const[0].replace('?',''))
            elif const_type == 'hardware_group':
                hardware_group.append(const[0].replace('?',''))
            elif const_type == 'planner':
                planner.append(const[0].replace('?',''))

    #-----------------------------------------------
    # Split primitives
    #-----------------------------------------------
    def split_primitives(self, group, primitives):
        if not type(group) is list:
            raise Exception('Error with primitives')
            
        parameters = []
        untyped_primitives = []
        
        while group:
            t = group.pop(0)
            if t == '-':
                if not untyped_primitives:
                    raise Exception('Unexpected hyphen in primitives')
                ptype = group.pop(0)
                action = {'name': ptype, 'parameters':[]}
                while untyped_primitives:
                    action['parameters'].append(untyped_primitives.pop(0))
                primitives.append(action)
            else:
                untyped_primitives.append(t)        

    #-----------------------------------------------
    # Split predicates
    #-----------------------------------------------

    def split_predicates(self, group, predicates):
        if group:        
            # check predicate condition
            t = group.pop(0)
            if t == "not" or t == "and" or t == "or":
                pred_list = [t]
                self.split_predicates(group.pop(0), pred_list)
                predicates.append(pred_list)

            elif t == "forall":
                #forall [type] [predicates]
                p = group.pop(0)
                pred_list = [t, self.split_parameters(p)]
                self.split_predicates(group.pop(0), pred_list)
                predicates.append(pred_list)
                
            elif t == "when":
                #when [condition_predicates] [predicates]
                p = group.pop(0)
                cond_pred = []
                self.split_predicates(p, cond_pred)
                pred_list = [t, cond_pred[0]]
                self.split_predicates(group.pop(0), pred_list)
                predicates.append(pred_list)

            elif t == '=':
                param_type = self.get_action_parameter_type(group[0])
                pred_list = ['equal']
                pred_list.append(self.struct_parameter(group[0], param_type))
                pred_list.append(self.struct_parameter(group[1], param_type))   
                predicates.append(pred_list)

            else:
                pred_list = []
                predicate = list(self.predicates[t])
                pred_list.append(t)
                for i, param in enumerate(predicate):
                    predicate[i] = {}          
                    predicate[i]['type'] = self.predicates[t][i]['type']              
                    predicate[i]['name'] = group[i]
                    pred_list.append(predicate[i])
                predicates.append(pred_list)     
        else:
            pass        

    #-----------------------------------------------
    # Split parameters
    #-----------------------------------------------

    def split_parameters(self, group):
        if not type(group) is list:
            raise Exception('Error with ' + name + ' parameters')
        parameters = []
        untyped_parameters = []
        p = group
        while p:
            t = p.pop(0)
            if t == '-':
                if not untyped_parameters:
                    raise Exception('Unexpected hyphen in ' + name + ' parameters')
                ptype = p.pop(0)
                while untyped_parameters:
                    parameters.append([untyped_parameters.pop(0), ptype])
            else:
                untyped_parameters.append(t)
        while untyped_parameters:
            pass
            
        return self.struct_parameters(parameters)

    def struct_parameters(self, parameters):
        param_list = []
        
        for param in parameters:
            param_dict = {}
            param_dict['name'] = param[0]
            param_dict['type'] = param[1]
            param_list.append(param_dict)
        
        return param_list

    def struct_parameter(self, param_name, param_type):
        return {'type': param_type, 'name': param_name}

    def get_action_parameter_type(self, parameter):
        action_params = self.actions[self.action]['parameters']
        for param in action_params:
            if param['name'] == parameter:
                return param['type']
        return ''


# ==========================================
# Main
# ==========================================
if __name__ == '__main__':
    import os
    script_dir = os.path.dirname(__file__) 
    rel_path = "../config/action_library.pddl"
    library_path = os.path.join(script_dir, rel_path)
    
    parser = PDDL_Parser()
    parser.parse_domain(library_path)

    #pprint.pprint(parser.scan_tokens(library_path))
    print('------------requirements----------------')
    print(parser.requirements)

    print('\n------------types----------------')
    print(parser.types)

    print('\n------------constants----------------')
    print(parser.constants)

    print('\n------------predicates----------------')
    for pred in parser.predicates.keys():
        print(pred, parser.predicates[pred])

    print('\n------------actions----------------')
    action_name = 'hold_object'
    print action_name
    print '\t', 'parameters: ', parser.actions[action_name]['parameters']
    print '\n\t', 'precondition: ', parser.actions[action_name]['precondition']
    print '\n\t', 'effect: ', parser.actions[action_name]['effect'] 
    print '\n\t', 'planner: ', parser.actions[action_name]['planner']
    print '\n\t', 'controller: ', parser.actions[action_name]['controller'] 
    print '\n\t', 'group: ', parser.actions[action_name]['group'] 
    print '\n\t', 'primitives: ', parser.actions[action_name]['primitives'] 