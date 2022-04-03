
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
    public class ContextSaverTest extends AbstractNodeMain {

      static ServiceClient<MonitorSimilarServiceRequest, MonitorSimilarServiceResponse> serviceClient;

    static Publisher<MainServiceRequest> requestor;

      static Log log;
    static int timeInterval=131;

      MonitorSimilarServiceRequest request;

      @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("rosjava_tutorial_custom_custom_services/client_service");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    try {
      log = connectedNode.getLog();

      serviceClient = connectedNode.newServiceClient("context_manager/monitor/service", MonitorSimilarService._TYPE);
        requestor = connectedNode.newPublisher("context_manager/main/reception", MainServiceRequest._TYPE);

      request = serviceClient.newMessage();
    } catch (ServiceNotFoundException e) {
      throw new RosRuntimeException(e);
    }


    while(true){
    

      sendMessage("on_Physical O1 O2 0 0 100");
       sleep(timeInterval);
       sendMessage("far O1 O2 0 0 100");
       sleep(timeInterval);
       sendMessage("near O1 O2 0 0 100");
       sleep(timeInterval);
       sendMessage("empty_hand H 0 0 0 100");
      sleep(timeInterval);
       sendMessage("closed_hand H 0 0 0 100");
      sleep(timeInterval);
       sendMessage("opened_hand H 0 0 0 100");
      sleep(timeInterval);
       sendMessage("full_hand H 0 0 0 100");
      sleep(timeInterval);
       sendMessage("toTheLeftOf O1 O2 0 0 100");
      sleep(timeInterval);

       sendMessage("toTheRightOf O1 O2 0 0 100");
      sleep(timeInterval);

       sendMessage("inFrontOf O1 O2 0 0 100");
      sleep(timeInterval);

       sendMessage("behind O1 O2 0 0 100");
      sleep(timeInterval);

       sendMessage("graspedBy O1 H 0 0 100");
      sleep(timeInterval);

       sendMessage("currentFingerPose O1 P 0 0 100");
      sleep(timeInterval);

       sendMessage("currentObjectPose O1 P 0 0 100");
      sleep(timeInterval);

       sendMessage("currentHandPose H P 0 0 100");
      sleep(timeInterval);

       sendMessage("handSize H S 0 0 100");
      sleep(timeInterval);

       sendMessage("objectSize O1 S 0 0 100");
      sleep(timeInterval);

       sendMessage("robotBodySize R S 0 0 100");
      sleep(timeInterval);

       sendMessage("currentRobotBodyPose R P 0 0 100");
      sleep(timeInterval);

       sendMessage("currentJointAngle J A 0 0 100");
      sleep(timeInterval);

       sendMessage("currentJointVelocity J V 0 0 100");
      sleep(timeInterval);

       sendMessage("currentJointEffort J E 0 0 100");
      sleep(timeInterval);

       sendMessage("locatedAt O1 P 0 0 100");
      sleep(timeInterval);

       sendMessage("belowOf O1 O2 0 0 100");
      sleep(timeInterval);

       sendMessage("aboveOf O1 O2 0 0 100");
      sleep(timeInterval);

       sendMessage("in_ContGeneric O1 O2 0 0 100");
      sleep(timeInterval);

       sendMessage("empty_container O1 0 0 0 100");
      sleep(timeInterval);


    }

}


  public void sendMessage(String str){
setMessage(request,str);

    serviceClient.call(request, new ServiceResponseListener<MonitorSimilarServiceResponse>() {
      @Override
      public void onSuccess(MonitorSimilarServiceResponse response) {
      if (response.getResponse().size()!=0){
        log.info("Service successed");
        for(int i=0;i<response.getResponse().size();i++)
          System.out.println(response.getResponse().get(i).getPredicate() + " " +response.getResponse().get(i).getParam1()+ " " +response.getResponse().get(i).getParam2()+ " " +response.getResponse().get(i).getParam3()+ " " +response.getResponse().get(i).getParam4());
        
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
