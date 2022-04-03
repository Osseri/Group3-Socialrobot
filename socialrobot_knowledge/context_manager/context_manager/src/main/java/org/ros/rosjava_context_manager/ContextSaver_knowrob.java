
/*
     * Copyright (C) 2011 Google Inc.
     *
     * Licensed under the Apache License, Version 2.0 (the "License"); you may not
     * use this file except in compliance with the License. You may obtain a copy of
     * the License at
     *
     * http://www.apache.org/licenses/LICENSE-2.0
     *
    * Unless required by applicable law or agreed to in writing, software
    * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
    * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
    * License for the specific language governing permissions and limitations under
    * the License.
    */

    package org.ros.rosjava_context_manager;

    import java.lang.*;
    import java.util.*;
    import java.lang.Integer;
    import org.ros.exception.RemoteException;
    import org.ros.exception.RosRuntimeException;
    import org.ros.exception.ServiceNotFoundException;
    import org.ros.namespace.GraphName;
    import org.ros.node.AbstractNodeMain;
    import org.ros.node.ConnectedNode;
    import org.ros.node.NodeMain;
    import org.ros.node.service.ServiceClient;
    import org.ros.node.service.ServiceResponseBuilder;
    import org.ros.node.service.ServiceResponseListener;
    import rosjava_custom_srv.*;
    import org.apache.commons.logging.Log;
    import org.ros.message.MessageListener;
    import org.ros.node.topic.*;
    import org.ros.internal.message.action.*;
    import org.jpl7.*;
    import java.util.*;
    
    /**
     * A simple {@link ServiceClient} {@link NodeMain}. The original code is created
     * by:
     *
     * @author damonkohler@google.com (Damon Kohler) The custom implementation is
     *         created by v.s.moisiadis@gmail.com(Vasileios Moisiadis)
     */
    public class ContextSaver_knowrob extends AbstractNodeMain {

      static ServiceClient<MonitorSimilarServiceRequest, MonitorSimilarServiceResponse> serviceClient;

    static Publisher<MainServiceRequest> requestor;

      static Log log;
    static int timeInterval=1310;
static ArrayList<String> properties = new ArrayList<String>();

      MonitorSimilarServiceRequest request;

      Map<String, String> prefixes = new HashMap<>();



      @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava_tutorial_custom_custom_services/client_service");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {

      prefixes.put("http://www.arbi.com/ontologies/arbi.owl","arbi");
      prefixes.put("http://knowrob.org/kb/comp_spatial.owl","comp_spatial");
      prefixes.put("http://knowrob.org/kb/comp_temporal.owl","comp_temporal");
      prefixes.put("http://knowrob.org/kb/computable.owl","computable");
      prefixes.put("http://purl.org/dc/elements/1.1/","dc");
      prefixes.put("http://knowrob.org/kb/knowrob.owl","knowrob");
      prefixes.put("http://www.w3.org/2002/07/owl","owl");
      prefixes.put("http://www.w3.org/1999/02/22-rdf-syntax-ns","rdf");
      prefixes.put("http://www.w3.org/2000/01/rdf-schema","rdfs");
      prefixes.put("http://knowrob.org/kb/srdl2-comp.owl","srdl2comp");
      prefixes.put("http://www.w3.org/2001/XMLSchema","xsd");

    try {
      log = connectedNode.getLog();

      serviceClient = connectedNode.newServiceClient("context_manager/monitor/service", MonitorSimilarService._TYPE);
        requestor = connectedNode.newPublisher("context_manager/main/reception", MainServiceRequest._TYPE);

      request = serviceClient.newMessage();
    } catch (ServiceNotFoundException e) {
      throw new RosRuntimeException(e);
    }



    

       sendMessage("rdf S P O 0 100");
      sleep(7000);

       sendMessage("rdf S P O 0 104");
      sleep(timeInterval);

       sendMessage("rdf S P O 0 105");
      sleep(timeInterval);
/*
      System.out.println(properties.size());

	for(int i =0 ;i<properties.size();i++){
          String qu = properties.get(i).replace("'","")+" S O 0 0 100";
          System.out.println(qu);
          sendMessage(qu);
          sleep(timeInterval);
         }



       sendMessage("currentObjectPerception S O 0 0 104");
       sleep(timeInterval);
*/
    }




  public void sendMessage(String str){
setMessage(request,str);

    serviceClient.call(request, new ServiceResponseListener<MonitorSimilarServiceResponse>() {
      @Override
      public void onSuccess(MonitorSimilarServiceResponse response) {
      if (response.getResponse().size()!=0){
        log.info("Service successed");

 for(int i=0;i<response.getResponse().size();i++){

System.out.println(response.getResponse().get(i).getPredicate() + " " +response.getResponse().get(i).getParam1()+ " " +response.getResponse().get(i).getParam2()+ " " +response.getResponse().get(i).getParam3()+ " " +response.getResponse().get(i).getParam4());


          String pp = response.getResponse().get(i).getParam2();
           for (Map.Entry<String, String> prefix : prefixes.entrySet()) {
            pp= pp.replace(prefix.getKey(),prefix.getValue()).replace("#",":");
          }
          if(properties.contains(pp))continue;

 



            properties.add(pp);
          

}
        
      }
      }

      @Override
      public void onFailure(RemoteException e) {
        log.info("Service failed");
        throw new RosRuntimeException(e);
      }
    });
  }

      public void setMessage(MonitorSimilarServiceRequest request, String query){
        String[] params = query.split(" ");
          request.setPredicate(params[0]);
          request.setParam1(params[1]);
          request.setParam2(params[2]);
          request.setParam3(params[3]);
          request.setParam4(params[4]);
          request.setStatus(Integer.parseInt(params[5]));//100);
          request.setManager("FSD");
      }


	public void sleep(int n){
		try{
			Thread.sleep(n);
		}catch(Exception e){}
	}
    }
