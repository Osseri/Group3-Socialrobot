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
public class SelvisClient extends AbstractNodeMain {

  static ServiceClient<SelvisServiceRequest, SelvisServiceResponse> serviceClient;

  static Log log;

  SelvisServiceRequest request;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("selvis/client_service");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    try {
      log = connectedNode.getLog();

      serviceClient = connectedNode.newServiceClient("selvis/service", SelvisService._TYPE);
      request = serviceClient.newMessage();
      sendMessage("selvis");
    } catch (ServiceNotFoundException e) {
      throw new RosRuntimeException(e);
    }



  }

  public void sendMessage(String str) {
    request.setFilepath(str);

    serviceClient.call(request, new ServiceResponseListener<SelvisServiceResponse>() {
      @Override
      public void onSuccess(SelvisServiceResponse response) {
        log.info(response.getSuccess());

        
      }

      @Override
      public void onFailure(RemoteException e) {
        log.info("Service failed");
        throw new RosRuntimeException(e);
      }
    });
  }


  public void sleep(int n) {
    try {
      Thread.sleep(n);
    } catch (Exception e) {
    }
  }
}
