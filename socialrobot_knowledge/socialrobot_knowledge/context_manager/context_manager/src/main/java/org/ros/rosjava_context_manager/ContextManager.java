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

import rosjava_custom_srv.*;
import rosjava_triple_msgs.*;

//import javafx.application.Application;

import java.lang.Integer;
import java.io.*;
import java.lang.Thread;
import org.apache.commons.logging.Log;
import org.ros.namespace.GraphName;
import org.ros.exception.*;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import java.io.Console;
import org.jpl7.*;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.topic.*;
import org.ros.message.MessageListener;

//import org.ros.web_data_send_action.*;

import java.util.*;

/**
 * A simple {@link ServiceServer} {@link NodeMain}. The original code is created
 * by:
 *
 * @author damonkohler@google.com (Damon Kohler) The custom implementation is
 *         created by v.s.moisiadis@gmail.com(Vasileios Moisiadis)
 */
public class ContextManager extends AbstractNodeMain {

  static int status = -200;
  static Publisher<Monitor> publisherForTM;
  static Subscriber<Monitor> subscriberForPM;
  static Subscriber<MonitorServiceRequest> receptionistForMonitor;
  static Subscriber<QueryServiceRequest> receptionistForQuery;
  static Publisher<QueryServiceResponse> qProviderForTM;
  static Publisher<QueryServiceResponse> qProviderForPM;
  static Publisher<MonitorServiceResponse> mProviderForTM;
  static Publisher<MonitorServiceResponse> mProviderForPM;
  static Subscriber<MainServiceRequest> receptionistForService;
  static Publisher<MainServiceRequest> tmpMainRequestGenerator;
  static Publisher<MonitorServiceRequest> tmpMonitorRequestGenerator;
  // static Subscriber<MonitorServiceResponse> tmpMonitorResponseReceiver;
  static Publisher<ContextOntologyMonitor> cContextProvider;
  static Publisher<LowLevelContextMonitor> lContextProvider;
  static Publisher<ContextReasoningMonitor> rContextProvider;

  static ServiceServer<MonitorSimilarServiceRequest, MonitorSimilarServiceResponse> mServiceServer;

  // static FSDUI fsdUI;

  static String graphFilePath = "/home/ubuntu/Desktop/test/social_ws/src/socialrobot/src/socialrobot_knowledge/context_manager/context_manager/src/main/java/org/ros/web_data_send_action/perceptionGraph.owl";
  static String knowrobFilePath = "/home/ubuntu/Desktop/test/social_ws/src/socialrobot/src/socialrobot_knowledge/cmProlog/knowrob_library/knowrob.owl";

  // WebDataSendAction WDSA;
  int flag = 0;

  // ***********************************add here by jaeyun
  // static Publisher<PredicateServiceResponse> pProviderForTM;
  // static Publisher<PredicateServiceResponse> pProviderForCM;
  // static Subscriber<PredicateServiceRequest> receptionistForPredicate;

  // static ServiceClient<MonitorServiceRequest, MonitorServiceResponse>
  // serviceClient;
  static ConnectedNode connectedNode;
  static Log log;

  final static int queueSize = 1000;
  static List<MonitorServiceResponse> tmpMonitorResponse = new ArrayList<MonitorServiceResponse>();
  int queryResultNum = -1;
  boolean fillingMonitorServiceResponse = false;

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("context_manager");
  }

  /*
   * public static void setMessage(Triple message, String param){ String[] params
   * = param.split(" "); message.setDomain(params[0]);
   * message.setProperty(params[1]); message.setRange(params[2]); }
   */

  public static void setMessage(Monitor message, String param) {
    param.replaceAll("\\'", "'");
    if (param.contains("monitored") && param.indexOf("monitored") != -1)
      param = param.substring(param.indexOf("monitored ") + "monitored ".length());

    String[] params = param.split(" ");
    message.setPredicate(params[0]);
    message.setParam1(params[1]);
    message.setParam2(params[2]);
    message.setParam3(params[3]);
    message.setParam4(params[4]);

    if (params[5].contains("'"))
      params[5] = params[5].substring(0, params[5].indexOf("'"));

    message.setStatus(Integer.parseInt(params[5]));
  }

  public static void setMessage(MonitorServiceRequest message, String param) {
    if (param.contains("monitored") && param.indexOf("monitored") != -1)
      param = param.substring(param.indexOf("monitored ") + "monitored ".length());

    String[] params = param.split(" ");
    message.setPredicate(params[0]);
    message.setParam1(params[1]);
    message.setParam2(params[2]);
    message.setParam3(params[3]);
    message.setParam4(params[4]);

    if (params[5].contains("'"))
      params[5] = params[5].substring(0, params[5].indexOf("'"));

    message.setStatus(Integer.parseInt(params[5]));
  }

  public static void copyMessageToRequest(Monitor message, MonitorServiceRequest request) {
    request.setPredicate(message.getPredicate());
    request.setParam1(message.getParam1());
    request.setParam2(message.getParam2());
    request.setParam3(message.getParam3());
    request.setParam4(message.getParam4());
    request.setStatus(message.getStatus());
  }

  public static void copyQueryToRequest(QueryServiceRequest qRequest, MonitorServiceRequest mRequest) {
    mRequest.setPredicate(qRequest.getQuery());
    mRequest.setManager(qRequest.getManager());
    mRequest.setStatus(101);
  }

  public static void copyQueryToRequest(QueryServiceRequest qRequest, MainServiceRequest mRequest) {

    List<String> temp = new ArrayList<String>();
    temp.add(qRequest.getQuery());
    mRequest.setPredicate(temp);

    int[] st = { 101 };
    mRequest.setStatus(st);

    temp = new ArrayList<String>();
    temp.add(qRequest.getManager());
    mRequest.setManager(temp);
  }

  public static void copyRequestToRequest(MonitorServiceRequest monitorRequest, MainServiceRequest mainRequest) {
    List<String> temp = new ArrayList<String>();
    temp.add(monitorRequest.getPredicate());
    mainRequest.setPredicate(temp);

    temp = new ArrayList<String>();
    temp.add(monitorRequest.getParam1());
    mainRequest.setParam1(temp);

    temp = new ArrayList<String>();
    temp.add(monitorRequest.getParam2());
    mainRequest.setParam2(temp);

    temp = new ArrayList<String>();
    temp.add(monitorRequest.getParam3());
    mainRequest.setParam3(temp);

    temp = new ArrayList<String>();
    temp.add(monitorRequest.getParam4());
    mainRequest.setParam4(temp);

    int[] st = { monitorRequest.getStatus() };
    mainRequest.setStatus(st);

    temp = new ArrayList<String>();
    temp.add(monitorRequest.getManager());
    mainRequest.setManager(temp);
    // mainRequest.setManager(monitorRequest.getManager());
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {

    ContextManager.connectedNode = connectedNode;
    ContextManager.log = connectedNode.getLog();

    try {

      publisherForTM = connectedNode.newPublisher("context_manager/monitor", Monitor._TYPE);

      // for Monitor
      mProviderForTM = connectedNode.newPublisher("context_manager/monitor/provision_for_tm",
          MonitorServiceResponse._TYPE); // km --> tm
      mProviderForPM = connectedNode.newPublisher("context_manager/monitor/provision_for_pm",
          MonitorServiceResponse._TYPE);

      // for Query
      qProviderForTM = connectedNode.newPublisher("context_manager/query/provision_for_tm", QueryServiceResponse._TYPE);
      qProviderForPM = connectedNode.newPublisher("context_manager/query/provision_for_pm", QueryServiceResponse._TYPE);

      // for Tmp
      tmpMainRequestGenerator = connectedNode.newPublisher("context_manager/main/reception", MainServiceRequest._TYPE);
      tmpMonitorRequestGenerator = connectedNode.newPublisher("context_manager/monitor/reception",
          MonitorServiceRequest._TYPE);
      // tmpMonitorRequestGenerator =
      // connectedNode.newPublisher("context_manager/monitor/provision_for_tm",MonitorServiceResponse._TYPE);
      // tmpMonitorResponseReceiver =
      // connectedNode.newSubscriber("context_manager/monitor/provision_for_cm",MonitorServiceInnerResponse._TYPE);

      // for Reception
      receptionistForMonitor = connectedNode.newSubscriber("context_manager/monitor/reception",
          MonitorServiceRequest._TYPE);// tm--> km
      receptionistForQuery = connectedNode.newSubscriber("context_manager/query/reception", QueryServiceRequest._TYPE);
      receptionistForService = connectedNode.newSubscriber("context_manager/main/reception", MainServiceRequest._TYPE);

      // for Predicate
      // pProviderForTM =
      // connectedNode.newPublisher("ContextManager/Predicate/ProvisionForTM",
      // PredicateServiceResponse._TYPE);
      // pProviderForCM =
      // connectedNode.newPublisher("ContextManager/Predicate/ProvisionForPM",
      // PredicateServiceResponse._TYPE);
      // receptionistForPr =
      // connectedNode.newSubscriber("ContextManager/Predicate/Reception",
      // PredicateServiceRequest._TYPE);

      // Graph Save
      cContextProvider = connectedNode.newPublisher("context_manager/context/provision_for_c",
          ContextOntologyMonitor._TYPE);
      lContextProvider = connectedNode.newPublisher("context_manager/context/provision_for_l",
          LowLevelContextMonitor._TYPE);
      rContextProvider = connectedNode.newPublisher("context_manager/context/provision_for_r",
          ContextReasoningMonitor._TYPE);

      mServiceServer = connectedNode.newServiceServer("context_manager/monitor/service", MonitorSimilarService._TYPE,
          new ServiceResponseBuilder<MonitorSimilarServiceRequest, MonitorSimilarServiceResponse>() {
            @Override
            public void build(MonitorSimilarServiceRequest request, MonitorSimilarServiceResponse response) {
              // Create an array with the size of request.getSize()
              tmpMonitorResponse = new ArrayList<MonitorServiceResponse>();
              MonitorServiceRequest mRequest = tmpMonitorRequestGenerator.newMessage();
              mRequest.setPredicate(request.getPredicate());
              mRequest.setParam1(request.getParam1());
              mRequest.setParam2(request.getParam2());
              mRequest.setParam3(request.getParam3());
              mRequest.setParam4(request.getParam4());
              mRequest.setStatus(request.getStatus());
              mRequest.setManager("ContextManager");
              tmpMonitorRequestGenerator.publish(mRequest);

              while (queryResultNum != 0)
                Thread.yield();
              queryResultNum = -1;
              response.setResponse(tmpMonitorResponse);

            }
          });

    } catch (Exception e) {
      throw new RosRuntimeException(e);
    }

    System.setOut(new PrintStream(System.out) {
      public void println(String s) {
        // System.out.println(s);
        if (s.contains("monitored")) {

          monitored(s);
        }
      }
    });

    receptionistForMonitor.addMessageListener(new MessageListener<MonitorServiceRequest>() {

      @Override
      public void onNewMessage(MonitorServiceRequest monitorRequest) {

        MainServiceRequest mainRequest = tmpMainRequestGenerator.newMessage();
        copyRequestToRequest(monitorRequest, mainRequest);
        tmpMainRequestGenerator.publish(mainRequest);
      }
    }, queueSize);

    receptionistForQuery.addMessageListener(new MessageListener<QueryServiceRequest>() {

      @Override
      public void onNewMessage(QueryServiceRequest qRequest) {

        MainServiceRequest mRequest = tmpMainRequestGenerator.newMessage();
        copyQueryToRequest(qRequest, mRequest);
        tmpMainRequestGenerator.publish(mRequest);
      }
    }, queueSize);

    receptionistForService.addMessageListener(new MessageListener<MainServiceRequest>() {

      @Override
      public void onNewMessage(MainServiceRequest request) {

        for (int i = 0; i < request.getPredicate().size(); i++) {
          String teststr = "";

          TempQuery query = new TempQuery();
          query.setPredicate(request.getPredicate().get(i));
          // System.out.println(query.getPredicate());

          teststr += request.getPredicate().get(i);

          if (request.getParam1().size() > i) {
            query.setParam1(request.getParam1().get(i));
            teststr += " " + request.getParam1().get(i);
          }
          if (request.getParam2().size() > i)
            query.setParam2(request.getParam2().get(i));
          if (request.getParam3().size() > i)
            query.setParam3(request.getParam3().get(i));
          if (request.getParam4().size() > i)
            query.setParam4(request.getParam4().get(i));
          if (request.getManager().size() > i)
            query.setManager(request.getManager().get(i));

          // System.out.println(request.getPredicate().size() + " " + teststr);
          // System.out.println(request.getStatus()[i]);
          // System.out.println(query);
          // if(query.getPredicate().equals("End"))break;
          log.info(request.getStatus()[i]);
          try {
            switch (request.getStatus()[i]) {

            case 8:
              monitor(query);
              break;
            case 4:
              rdf_retract(query);
              break;
            case 3:
              rdf_assert(query);
              break;
            case 0:
              regist1(query);
              break;
            case 2:
              regist2(query);
              break;
            case -1:
              cancel1(query);
              break;
            case -2:
              cancel2(query);
              break;
            case 99:
              initialize();
              break;
            case 100:
              query(query);
              break;
            case 101:
              query2(query);
              break;
            case 102:
              query3(query);
              break;
            case 103:
              clearFSD();
              break;
            case 104:
              saveGraph(graphFilePath);
              break;
            case 105:
              LowLevelContextMonitor llcm = lContextProvider.newMessage();
              ContextOntologyMonitor com = cContextProvider.newMessage();

              llcm.setPath(graphFilePath);
              com.setPath(knowrobFilePath);

              lowLevelContextMonitor(llcm);
              contextOntologyMonitor(com);
              break;

            // default:
            // query(request);
            }
          } catch (Exception e) {
            e.printStackTrace();
          }

          if (status == 9) {
            ContextManager.log.info("<==============receive success==============>");
            /*
             * QueryServiceResponse response = qProviderForPM.newMessage();
             * response.setResult("Done");
             * 
             * if (request.getManager().equals("ContextManager"))
             * qProviderForPM.publish(response); else if
             * (request.getManager().equals("TaskManager"))
             * qProviderForTM.publish(response);
             */
          } else if (status != 3 && status != 4) {
            MonitorServiceResponse response = mProviderForPM.newMessage();
            response.setStatus(status);

            if (request.getManager().equals("ContextManager"))
              mProviderForPM.publish(response);
            else if (request.getManager().equals("TaskManager"))
              mProviderForTM.publish(response);
          }
          status = 9;
        }
      }
    }, queueSize);

    /*
     * tmpMonitorResponseReceiver.addMessageListener(new
     * MessageListener<MonitorServiceInnerResponse>() {
     * 
     * @Override public void onNewMessage(MonitorServiceInnerResponse response) {
     * 
     * //String ans = Parser.text_transPredicate(response);
     * 
     * //if (tmpMonitorResponse.size()!=0 &&
     * !tmpMonitorResponse.get(0).getPredicate().equals(response.getPredicate())){
     * // fillingMonitorServiceResponse=false; // tmpMonitorResponse.clear(); //}
     * //fillingMonitorServiceResponse=true;
     * 
     * tmpMonitorResponse=response.getResponse(); //queryResultNum--;
     * 
     * 
     * } }, queueSize);
     */

    /*
     * connectedNode.newServiceServer("context_manager/monitor/service",
     * MonitorSimilarService._TYPE, new
     * ServiceResponseBuilder<MonitorSimilarServiceRequest,
     * MonitorSimilarServiceResponse>() {
     * 
     * @Override public void build(MonitorSimilarServiceRequest request,
     * MonitorSimilarServiceResponse response) { //Create an array with the size of
     * request.getSize() tmpMonitorResponse.clear(); MonitorServiceRequest mRequest
     * = tmpMonitorRequestGenerator.newMessage();
     * mRequest.setPredicate(request.getPredicate());
     * mRequest.setParam1(request.getParam1());
     * mRequest.setParam2(request.getParam2());
     * mRequest.setParam3(request.getParam3());
     * mRequest.setParam4(request.getParam4());
     * mRequest.setStatus(request.getStatus());
     * mRequest.setManager("ContextManager");
     * tmpMonitorRequestGenerator.publish(mRequest);
     * 
     * while(queryResultNum!=0)log.info(queryResultNum);
     * response.setResponse(tmpMonitorResponse); } });
     */

    // fsdUI = new FSDUI();
    // Application.launch(fsdUI.getClass());

  }

  public void initialize() {
    String basePath = "/home/ubuntu/Desktop/test/social_ws/src/socialrobot/src/socialrobot_knowledge/cmProlog/prolog/";

    String initialize = "consult('" + basePath + "init.pl')";
    Query query = new Query(initialize);
    ContextManager.log.info(initialize + " " + (query.hasSolution() ? "successed" : "failed"));

    /*
     * initialize = "currentObjectPerception(Object, CurrentPerception)"; String
     * response = ""; query = new Query(initialize); Map<String,Term>[] getResponse
     * = query.allSolutions(); ContextManager.log.info("Cancellation------" + " " +
     * getResponse.length); for(int i = 0; i < getResponse.length; i++){ response +=
     * getResponse[i]; ContextManager.log.info("Cancellation" + " " + response); }
     */
    // ContextManager.log.info("Cancellation" + " " + response);

    initialize = "consult('" + basePath + "monitor.pl')";
    query = new Query(initialize);
    ContextManager.log.info(initialize + " " + (query.hasSolution() ? "successed" : "failed"));

    initialize = "start_monitor";
    query = new Query(initialize);
    ContextManager.log.info(initialize + " " + (query.hasSolution() ? "successed" : "failed"));

  }

  /*
   * public void query(MonitorServiceRequest request) {
   * 
   * Query query = new Query(request.getPredicate());
   * 
   * status = query.hasSolution() ? 9 : -99;
   * 
   * ContextManager.log.info(request.getPredicate() + " " + (query.hasSolution() ?
   * "successed" : "failed")); }
   */
  public void rdf_assert(TempQuery request) {// MonitorServiceRequest request) {

    String rdf_assert = "rdf_assert(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2() + ", perception)";

   /* if (request.getParam4() == null || request.getParam4().equals(""))
      rdf_assert += ",perception)";
    else
      rdf_assert += "," + request.getParam4() + ")";*/

    Query query = new Query(rdf_assert);

    status = query.hasSolution() ? 3 : -99;

    ContextManager.log.info(rdf_assert + " " + (query.hasSolution() ? "successed" : "failed"));

    // if(flag == 0){
    // WDSA = new WebDataSendAction();
    // WDSA.SD = WDSA.GGDA.SD;
    // WDSA.GGDA.start();
    // flag = 1;
    // }

    // ContextManager.log.info("@\n@\n@\n@\n@\n@\nprint log WDSA.SD.COMD.filepath:
    // "+WDSA.SD.COMD.filepath);
    // // for(int i = 0; i < WDSA.GGDA.graphs.length;i++)
    // // ContextManager.log.info("WDSA.graphs: "+WDSA.graphs[i]);
    // ContextManager.log.info("WDSA.SD.COMD.triples: "+WDSA.SD.COMD.triples);
    // ContextManager.log.info("WDSA.SD.COMD.classes: "+WDSA.SD.COMD.classes);
    // ContextManager.log.info("WDSA.SD.COMD.objectProperties:
    // "+WDSA.SD.COMD.objectProperties);
    // ContextManager.log.info("WDSA.SD.COMD.individuals:
    // "+WDSA.SD.COMD.individuals);
    // ContextManager.log.info("WDSA.SD.COMD.datatypeProperties:
    // "+WDSA.SD.COMD.datatypeProperties);
    // ContextManager.log.info("WDSA.SD.LCMD.filepath: "+WDSA.SD.LCMD.filepath);
    // ContextManager.log.info("WDSA.SD.LCMD.humanperception:
    // "+WDSA.SD.LCMD.humanperception);
    // ContextManager.log.info("WDSA.SD.LCMD.objectperception:
    // "+WDSA.SD.LCMD.objectperception);
    // ContextManager.log.info("WDSA.SD.LCMD.robotperception:
    // "+WDSA.SD.LCMD.robotperception);
    // ContextManager.log.info("WDSA.SD.LCMD.triples: "+WDSA.SD.LCMD.triples);

  }

  public void rdf_retract(TempQuery request) {// MonitorServiceRequest request){
    String rdf_retract = "rdf_retractall(" + request.getParam1() + "," + request.getPredicate() + ","
        + request.getParam2() + ")";

    Query query = new Query(rdf_retract);

    status = query.hasSolution() ? 4 : -99;

    // ContextManager.log.info(rdf_retract + " " + (query.hasSolution() ?
    // "successed" : "failed"));
    ContextManager.log.info("retract" + " " + (query.hasSolution() ? "successed" : "failed"));
  }

  public void monitor(TempQuery request) {// MonitorServiceRequest request) {
    String cancel = "rdf_retractall(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",D)";

    Query query = new Query(cancel);

    status = query.hasSolution() ? 8 : -99;

    String monitor = "rdf_assert(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",perception)";
    // rdf_assert(S, P, O, perception)
    query = new Query(monitor);
    ContextManager.log.info(monitor + " " + (query.hasSolution() ? "successed" : "failed"));

  }

  public void monitored(String params) {
    ContextManager.log.info("Monitored " + params);
    String data = null;
    Monitor message = publisherForTM.newMessage();
    setMessage(message, params);

    ContextManager.log.info(message.getPredicate() + " " + message.getParam1() + " " + message.getParam2() + " "
        + message.getParam3() + " " + message.getParam4());

    data = "[";

    String rdf = "rdf(" + message.getParam1() + "," + message.getPredicate() + "," + message.getParam2() + ")";
    Query query = new Query(rdf);

    status = query.hasSolution() ? 1 : -99;

    ContextManager.log.info(message.getPredicate() + " " + (query.hasSolution() ? "successed" : "failed"));

    String v = null;
    String v2 = null;

    if (Character.isUpperCase(message.getParam1().charAt(0)))
      v = message.getParam1();
    else if (Character.isUpperCase(message.getParam2().charAt(0)))
      v = message.getParam2();

    if (query.hasSolution()) {
      Map<String, Term>[] solutions = query.allSolutions();

      for (int i = 0; i < solutions.length; ++i) {
        data += solutions[i].get(v);
        // data += ":";
        // data += solutions[i].get(v2);
        // ContextManager.log.info(data);
        if (i + 1 < solutions.length)
          data += ",";
      }
    }

    data += "]";

    if (message.getStatus() == 2) {
      MonitorServiceRequest request = tmpMonitorRequestGenerator.newMessage();
      copyMessageToRequest(message, request);
      request.setStatus(-2);
      tmpMonitorRequestGenerator.publish(request);// cancel2(request);
    }

    if (Character.isUpperCase(message.getParam1().charAt(0)))
      message.setParam1(data);
    else if (Character.isUpperCase(message.getParam2().charAt(0)))
      message.setParam2(data);

    publisherForTM.publish(message);
  }

  public void regist1(TempQuery request) {// MonitorServiceRequest request) {
    String cancel = "rdf_retractall(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",D)";

    Query query = new Query(cancel);

    query.hasSolution();
    String regist = "assert(monitor(assert(" + request.getParam1() + "," + request.getPredicate() + ","
        + request.getParam2() + ",D)) :- (" + "jpl_get('java.lang.System',out,O)," + "jpl_call(O,println,[\"'monitored "
        + request.getPredicate() + " " + request.getParam1() + " " + request.getParam2() + " " + request.getParam3()
        + " " + request.getParam4() + " " + 1 + "'\"],Result), rdf_retractall(" + request.getParam1() + ","
        + request.getPredicate() + "," + request.getParam2() + ",D)" + "))";
    query = new Query(regist);

    status = query.hasSolution() ? 0 : -99;

    ContextManager.log.info("Registration" + " " + (query.hasSolution() ? "successed" : "failed"));
  }

  public void regist2(TempQuery request) {// MonitorServiceRequest request) {
    String cancel = "rdf_retractall(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",D)";

    Query query = new Query(cancel);
    query.hasSolution();
    String regist = "assert(monitor(assert(" + request.getParam1() + "," + request.getPredicate() + ","
        + request.getParam2() + ",D)) :- (" + "jpl_get('java.lang.System',out,O)," + "jpl_call(O,println,[\"'monitored "
        + request.getPredicate() + " " + request.getParam1() + " " + request.getParam2() + " " + request.getParam3()
        + " " + request.getParam4() + " " + 2 + "'\"],Result), rdf_retractall(" + request.getParam1() + ","
        + request.getPredicate() + "," + request.getParam2() + ",D)" + "))";
    query = new Query(regist);

    status = query.hasSolution() ? 2 : -99;

    ContextManager.log.info("Registration" + " " + (query.hasSolution() ? "successed" : "failed"));
  }

  public void cancel1(TempQuery request) {// MonitorServiceRequest request) {
    String cancel = "rdf_retractall(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",D)";
    Query query = new Query(cancel);
    query.hasSolution();
    cancel = "retract(monitor(assert(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",D)) :- (" + "jpl_get('java.lang.System',out,O)," + "jpl_call(O,println,[\"'monitored "
        + request.getPredicate() + " " + request.getParam1() + " " + request.getParam2() + " " + request.getParam3()
        + " " + request.getParam4() + " " + 1 + "'\"],Result), rdf_retractall(" + request.getParam1() + ","
        + request.getPredicate() + "," + request.getParam2() + ",D)" + "))";

    query = new Query(cancel);

    status = query.hasSolution() ? -1 : -99;

    ContextManager.log.info("Cancellation" + " " + (query.hasSolution() ? "successed" : "failed"));
  }

  public void cancel2(TempQuery request) {// MonitorServiceRequest request) {
    String cancel = "rdf_retractall(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",D)";
    Query query = new Query(cancel);
    query.hasSolution();

    cancel = "retract(monitor(assert(" + request.getParam1() + "," + request.getPredicate() + "," + request.getParam2()
        + ",D)) :- (" + "jpl_get('java.lang.System',out,O)," + "jpl_call(O,println,[\"'monitored "
        + request.getPredicate() + " " + request.getParam1() + " " + request.getParam2() + " " + request.getParam3()
        + " " + request.getParam4() + " " + 2 + "'\"],Result), rdf_retractall(" + request.getParam1() + ","
        + request.getPredicate() + "," + request.getParam2() + ",D)" + "))";

    query = new Query(cancel);

    status = query.hasSolution() ? -2 : -99;

    ContextManager.log.info("Cancellation" + " " + (query.hasSolution() ? "successed" : "failed"));
  }

  public void query(TempQuery request) {// MonitorServiceRequest request) {
    // String predicateQuery = request.getPredicate() + "(" + request.getParam1;

    String transQuery = Parser.transPredicateQuery(request); // ok
    MonitorServiceResponse response = mProviderForTM.newMessage();

    ContextManager.log.info("receive Query: " + transQuery);

    long beforeTime = System.currentTimeMillis();
    Query query = new Query(transQuery);
    Map<String, Term>[] preResponse = query.allSolutions();
    long afterTime = System.currentTimeMillis();

    if (request.getManager().equals("ContextManager"))
      queryResultNum = preResponse.length;

    for (int i = 0; i < preResponse.length; i++) {
      // ContextManager.log.info("number iiiiiiiiiiiiiiiiiisssssssssssssssss "+i);
      // ContextManager.log.info("들어옴");
      // MonitorServiceResponse response = mProviderForTM.newMessage();
      Parser.setResponse(request, preResponse[i], response);
      response.setManager("ContextManager");

      if (request.getManager().equals("ContextManager")) {
        tmpMonitorResponse.add(response);
        queryResultNum--;

        // make graph
        MonitorServiceRequest mRequest = tmpMonitorRequestGenerator.newMessage();
        mRequest.setPredicate(response.getPredicate());
        mRequest.setParam1(response.getParam1());
        mRequest.setParam2(response.getParam2());
        mRequest.setParam3(response.getParam3());
        mRequest.setParam4("highLevel");
        mRequest.setStatus(3);
        mRequest.setManager("ContextManager");
        tmpMonitorRequestGenerator.publish(mRequest);

      }

      if (preResponse.length == 0) {
        response.setParam1("[None]");
        response.setManager("ContextManager");
      }

      mProviderForTM.publish(response);

    }

    // test
    String transResponse = Parser.transPredicateResponse(request, query.allSolutions());

    ContextReasoningMonitor com = rContextProvider.newMessage();
    com.setSender("TM");
    com.setQuery(transQuery);
    com.setResponse(transResponse);
    com.setResponseTime(Integer.toString((int) (afterTime - beforeTime)) + "ms");
    com.setPredicates(request.getPredicate());
    rContextProvider.publish(com);

    ContextManager.log.info("all response: " + transResponse);
    // ContextManager.log.info("predicate:"+response.getPredicate()+",Param1:"+response.getParam1());
    ContextManager.log.info("<========================= Success =========================>");
  }

  public void query2(TempQuery request) {// MonitorServiceRequest request){
    ContextManager.log.info("receive Query: " + request.getPredicate());
    Query query = new Query(request.getPredicate());
    status = 101;

    String results = "";
    Map<String, Term> solutions[] = query.allSolutions();
    for (int i = 0; i < solutions.length; ++i) {
      Map<String, Term> solution = solutions[i];
      String result = "";
      if (solution.isEmpty()) {
        result = "True";
      } else {
        if (i != 0) {
          result += "-/#/-";
        }
        Set<String> keys = solution.keySet();
        Iterator<String> keyIter = keys.iterator();
        while (keyIter.hasNext()) {
          String key = keyIter.next();
          Term value = solution.get(key);
          if (Util.isList(value)) {
            result += key + "-/@/-" + listToString(value);
          } else {
            result += key + "-/@/-" + value.toString();
          }

          if (keyIter.hasNext()) {
            result += "-/-/-";
          }
        }
      }

      results += result;
    }

    if (results.equals("")) {
      results = "False";
    }
    QueryServiceResponse response = qProviderForTM.newMessage();
    response.setResult(results);
    qProviderForTM.publish(response);
    // fsdUI.updateFSD(results.toString());
  }

  public void query3(TempQuery request) {// MonitorServiceRequest request){
    Query query = new Query(request.getPredicate());
    status = 102;
    // FSD fsd;
    String data;

    // System.out.println(request);

    Map<String, Term> solutions[] = query.allSolutions();
    // if(solutions.length==0){
    // fsd = new FSD();
    // fsd.setPredicate(request.getPredicate());
    // fsd.setParam1("False");
    // fsdUI.updateFSD(fsd);
    // }
    for (int i = 0; i < solutions.length; ++i) {

      Map<String, Term> solution = solutions[i];

      Set<String> keys = solution.keySet();
      Iterator<String> keyIter = keys.iterator();

      // fsd = new FSD();
      // fsd.setPredicate(request.getPredicate().split("\\(")[0]);

      for (int j = 0; j < keys.size(); j++) {

        String key = keyIter.next();
        Term value = solution.get(key);
        if (Util.isList(value)) {
          data = listToString(value);
          data = data.replace("-/!/-", ",");
        } else {
          data = value.toString();
        }

        switch (j) {
        case 0:
          // fsd.setParam1(data);break;
        case 1:
          // fsd.setParam2(data);break;
        case 2:
          // fsd.setParam3(data);break;
        case 3:
          // fsd.setParam4(data);break;
        default:
          ;
        }

      }

      // fsdUI.updateFSD(fsd);
    }
  }

  public void clearFSD() {
    // fsdUI.clearFSD();
  }

  private String listToString(Term list) {
    String result = "[";

    Term[] terms = Util.listToTermArray(list);
    for (int i = 0; i < terms.length; ++i) {
      Term term = terms[i];
      if (Util.isList(term)) {
        result += listToString(term);
      } else {
        result += term.toString();
      }
      if (i != terms.length - 1) {
        result += "-/!/-";
      }
    }
    return result + "]";
  }

  public void saveGraph(String filepath) {
    String getGraphData = "rdf_save('" + filepath + "', [+graph(perception)]).";log.info(getGraphData);
    Query q = new Query(getGraphData);
    ContextManager.log.info(q.hasSolution() ? "save graph succeeded" : "save graph failed");
    // SD.COMD.filepath = filepath;
    // SD.LCMD.filepath = filepath;
    // q = new Query("rdf_graph(G).");
    // Map<String, Term>[] Obj = q.allSolutions();
    // for(int i = 0 ; i < Obj.length; i++)
    // graphs[i] = Obj[i].get("G").toString();
  }

  public void lowLevelContextMonitor(LowLevelContextMonitor llcm) {

    int tCount = 0;
    int oCount = 0;
    int rCount = 0;
    int hCount = 0;

    String getGraphDataEdge = null;
    String tripleNum = "0";
    getGraphDataEdge = "rdf_graph_property(perception, triples(Count)).";
    Query q = new Query(getGraphDataEdge);
    Map<String, Term>[] Obj = q.allSolutions();

    if (Obj.length != 0)
      tripleNum = Obj[0].get("Count").toString();
    tCount = Integer.parseInt(tripleNum);

    // System.out.println("getGraphDataTriple: "+tCount);

    // SD.LCMD.triples = Integer.toString(tripleNum);

    String getGraphDataObject = null;
    getGraphDataObject = "rdf(ObjectPerception, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#VisualObjectPerception',perception).";
    q = new Query(getGraphDataObject);
    while (q.hasMoreSolutions()) {
      q.nextSolution();
      oCount++;
    }
    // System.out.println("getGraphDataObject:"+oCount);
    // count=0;

    // SD.LCMD.objectperception = Integer.toString((int) count);

    String getGraphDataRobot = null;
    getGraphDataRobot = "(rdfs_individual_of(RobotPerception, 'http://knowrob.org/kb/knowrob.owl#Proprioception');rdfs_individual_of(RobotPerception, 'http://knowrob.org/kb/knowrob.owl#VisualRobotPerception')),(rdf(RobotPerception,_,_,perception);rdf(_,_,RobotPerception,perception)).";
    q = new Query(getGraphDataRobot);

    while (q.hasMoreSolutions()) {
      q.nextSolution();
      rCount++;
    }
    // System.out.println("getGraphDataRobot:"+rCount);
    // count=0;
    // SD.LCMD.robotperception = Integer.toString((int) count);

    String getGraphDataHuman = null;
    getGraphDataHuman = "rdf(HumanPerception, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type', 'http://knowrob.org/kb/knowrob.owl#VisualHumanPerception',perception),(rdf(HumanPerception,_,_,perception);rdf(_,_,HumanPerception,perception)).";
    q = new Query(getGraphDataHuman);

    while (q.hasMoreSolutions()) {
      q.nextSolution();
      hCount++;
    }
    // System.out.println("getGraphDataHuman:"+hCount);

    // SD.LCMD.humanperception = Integer.toString((int) count);

    llcm.setTriples(Integer.toString(tCount));
    llcm.setObjects(Integer.toString(oCount));
    llcm.setRobots(Integer.toString(rCount));
    llcm.setHumans(Integer.toString(hCount));
    lContextProvider.publish(llcm);

  }

  public void contextOntologyMonitor(ContextOntologyMonitor com) {
    int tCount = 0;
    int cCount = 0;
    int iCount = 0;
    int oCount = 0;
    int dCount = 0;

    String getGraphDataEdge = null;
    String tripleNum = "0";
    getGraphDataEdge = "rdf_graph_property('file://" + knowrobFilePath + "', " + "triples(Count))."; // file:///home/ubuntu/cmProlog/knowrob_library/knowrob.owl',
                                                                                                     // triples(Count)).";
    Query q = new Query(getGraphDataEdge);
    Map<String, Term>[] Obj = q.allSolutions();

    if (Obj.length != 0)
      tripleNum = Obj[0].get("Count").toString();
    tCount = Integer.parseInt(tripleNum);

    // System.out.println("getGraphDataTriple: "+tripleNum);

    // SD.COMD.triples = Integer.toString(tripleNum);

    HashSet classSet = new HashSet();
    String getGraphClasses = null;
    getGraphClasses = "(rdf(_,'http://www.w3.org/2000/01/rdf-schema#subClassOf',S,'file://" + knowrobFilePath
        + "');rdf(S,'http://www.w3.org/2000/01/rdf-schema#subClassOf',_,'file://" + knowrobFilePath + "')).";

    q = new Query(getGraphClasses);
    Map<String, Term>[] t = q.allSolutions();
    for (int i = 0; i < t.length; i++) {
      classSet.add(t[i].get("S").toString());
    }
    // System.out.println("getGraphClasses:"+ClassSet.size());
    // System.out.println("Classes:"+ClassSet);

    cCount = classSet.size();
    // SD.COMD.classes = Integer.toString(ClassSet.size());

    HashSet individualSet = new HashSet();
    String getGraphDataIndividual = null;
    getGraphDataIndividual = "rdf_reachable(S,'http://www.w3.org/1999/02/22-rdf-syntax-ns#type','http://www.w3.org/2002/07/owl#NamedIndividual'),rdf(S,_,_,'file://"
        + knowrobFilePath + "').";
    q = new Query(getGraphDataIndividual);

    Map<String, Term>[] s = q.allSolutions();
    for (int i = 0; i < s.length; i++) {
      individualSet.add(s[i].get("S").toString());
    }
    // System.out.println("getGraphDataIndividual:"+individualSet.size());

    // SD.COMD.individuals = Integer.toString(individualSet.size());
    iCount = individualSet.size();

    HashSet objectPropertySet = new HashSet();
    String getGraphDataObjectProperty = null;
    getGraphDataObjectProperty = "rdf_reachable(P, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type','http://www.w3.org/2002/07/owl#ObjectProperty'),rdf(P,_,_,'file://"
        + knowrobFilePath + "').";
    q = new Query(getGraphDataObjectProperty);

    Map<String, Term>[] w = q.allSolutions();

    for (int i = 0; i < w.length; i++) {
      objectPropertySet.add(w[i].get("P").toString());
    }
    // System.out.println("getGraphDataObjectProperty:"+objectPropertySet.size());

    // SD.COMD.objectProperties= Integer.toString(objectPropertySet.size());
    oCount = objectPropertySet.size();

    HashSet dataPropertySet = new HashSet();
    String getGraphDataDataProperty = null;
    getGraphDataDataProperty = "rdf_reachable(P, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#type','http://www.w3.org/2002/07/owl#DatatypeProperty'),rdf(P,_,_,'file://"
        + knowrobFilePath + "').";
    q = new Query(getGraphDataDataProperty);

    Map<String, Term>[] l = q.allSolutions();

    for (int i = 0; i < l.length; i++) {
      dataPropertySet.add(l[i].get("P").toString());
    }
    // System.out.println("getGraphDataDataProperty:"+dataPropertySet.size());

    // SD.COMD.datatypeProperties = Integer.toString(dataPropertySet.size());
    dCount = dataPropertySet.size();

    com.setTriples(Integer.toString(tCount));
    com.setClasses(Integer.toString(cCount));
    com.setIndividuals(Integer.toString(iCount));
    com.setObjectProperties(Integer.toString(oCount));
    com.setDatatypeProperties(Integer.toString(dCount));

    cContextProvider.publish(com);

  }

  public void sleep(int n) {
    try {
      Thread.sleep(n);
    } catch (Exception e) {
    }
  }

}