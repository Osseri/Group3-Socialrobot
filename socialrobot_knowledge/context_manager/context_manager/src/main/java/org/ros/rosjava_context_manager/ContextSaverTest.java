/*
 * Copyright (C) 2014 ailab.
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

import java.nio.file.*;
//import static org.junit.Assert.assertThat;

import java.util.*;

/**
 * A simple {@link Subscriber} {@link NodeMain}.
 */
public class ContextSaverTest extends AbstractNodeMain {

    final static int queueSize = 1000;


    static List<String> fSubject = new ArrayList<String>();
    static List<String>  fProperty = new ArrayList<String>();
    static List<String>  fObject = new ArrayList<String>();
    static List<String>  fGraph = new ArrayList<String>();
    static int[] fStatus = new int[1000];
    static int fInd;
    static List<String>  fManager = new ArrayList<String>();



    static Publisher<MonitorServiceRequest> FSDRequestor;

    static Publisher<MainServiceRequest> requestor;
    static Subscriber<LowLevelContextMonitor> receiver;
    static Publisher<LowLevelContextMonitor> provider;


    static String basePath;
    static String graphDirectoryPath;
    static String graphFileName = "perceptionGraph_changed.owl";
    static String graphFileName2 = "perceptionGraph_changed2.owl";

    static Log log;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("context_saver");
    }

    @Override
	public void onStart(ConnectedNode connectedNode) {
        log = connectedNode.getLog();

        basePath = new File(ContextManager.class.getProtectionDomain().getCodeSource().getLocation().getPath()).getAbsolutePath();
        basePath = basePath.substring(0,basePath.indexOf("context_manager"));

        graphDirectoryPath = basePath + "context_manager/context_manager/src/main/java/org/ros/web_data_send_action";

        FSDRequestor = connectedNode.newPublisher("context_manager/monitor/reception", MonitorServiceRequest._TYPE);


        requestor = connectedNode.newPublisher("context_manager/main/reception", MainServiceRequest._TYPE);

		
		receiver = connectedNode.newSubscriber("context_manager/context/provision_for_l", LowLevelContextMonitor._TYPE);
		provider = connectedNode.newPublisher("context_manager/web/provision_for_l", LowLevelContextMonitor._TYPE);

	
		
		receiver.addMessageListener(new MessageListener<LowLevelContextMonitor>() {
			@Override
			public void onNewMessage(LowLevelContextMonitor response) {
                File outFile = new File(graphDirectoryPath, graphFileName);
                File outFile2 = new File(graphDirectoryPath, graphFileName2);

                ///home/ubuntu/Desktop/test/social_ws/src/socialrobot/src/socialrobot_knowledge/context_manager/context_manager/src/main/java/org/ros/web_data_send_action/perceptionGraph_changed.owl
        //==========================//
        // 텍스트 파일 읽기
        //==========================//
/*
        String texts = "";
        BufferedReader br = null;
        BufferedWriter bw = null;
        try {
            br = new BufferedReader(new FileReader(response.getPath()));
            bw = new BufferedWriter(new FileWriter(outFile));

            String line=null;
            log.info("Modifying graph file");
            while ((line = br.readLine()) != null) {
                if(line.contains("<owl:imports rdf:resource=\"package://knowrob_common/owl/knowrob.owl\"/>")){
                  line = "";
                  //log.info("line removed 1");
                }
                if(line.contains("<owl:imports rdf:resource=\"package://knowrob_common/owl/rdf-schema.xml\"/>")){
                  line = "";
                  //log.info("line removed 2");
                }
                bw.write(line+"\n");
                
            }
            log.info("Modifying graph file done");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }finally {
            if(br != null) try {br.close(); } catch (IOException e) {}
        }

        //==========================//
        // 텍스트 파일 쓰기
        //==========================//
        
        try {
            
            bw.flush();
            bw.close();

        } catch (IOException e) {
            e.printStackTrace();
        }finally {
            if(bw != null) try {bw.close(); 
                /*Path source = Paths.get(graphDirectoryPath+graphFileName);
                Files.move(source, source.resolveSibling(graphDirectoryPath+"perceptionGraph_changed2.owl"),
                StandardCopyOption.REPLACE_EXISTING);
                //String cpFilePath = graphDirectoryPath+"perceptionGraph_changed2.owl";
                //Path copied = Paths.get(cpFilePath);
                //Path originalPath = Paths.get(graphDirectoryPath+graphFileName);
                //Files.copy(originalPath, copied, StandardCopyOption.REPLACE_EXISTING);
  
                //assertThat(copied).exists();
                //assertThat(Files.readAllLines(originalPath).equals(Files.readAllLines(copied)));
            response.setPath(graphDirectoryPath+graphFileName2);

            provider.publish(response);} catch (IOException e) {}
        }*/

				
      
		
			}
		});
        
        
        try{
        int t = 0;

        MainServiceRequest mRequest = null;
        List<String> req = new ArrayList<String>();
        int[] st = {0};


		while(true){
                sleep(1000);
                t+=1000;

				if(t%6000 == 0){
				 mRequest = requestor.newMessage();
			     st[0] = 104;
				 req =  new ArrayList<String>();
				 req.add("Graph Save");
				 mRequest.setPredicate(req);
				 mRequest.setStatus(st);
				 req =  new ArrayList<String>();
				 req.add("ContextManager");
				 mRequest.setManager(req);
                 requestor.publish(mRequest);

                 log.info("Graph Save request");
                 
                 //t=0;
                }
                if(t%6000 == 0){
                    sendFSD();
                }



				mRequest = requestor.newMessage();
			    st[0] = 105;
				req =  new ArrayList<String>();
				req.add("Send context");
				mRequest.setPredicate(req);
				mRequest.setStatus(st);
				req =  new ArrayList<String>();
				req.add("ContextManager");
				mRequest.setManager(req);
                requestor.publish(mRequest);
                
                log.info("Context sended");

                
			}
		}catch(Exception e)
    {
        e.printStackTrace();
    }

    }

    public void sendFSD(){
        try{

            // String message="";
            MonitorServiceRequest request = FSDRequestor.newMessage();
            String query = "";
            //message = queryInput.nextLine();
            //setMessage(request,message);

        /*
            //start here!!!
            query = "obstruct Obstacle Target 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "reachable Body Object 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "lookAt Robot Object 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);



            //Perceptual
            query = "far Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "near Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "leftOf Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "rightOf Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);
*/
            query = "inFrontOf Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "behind Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

/*
            query = "belowOf Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "aboveOf Object1 Object2 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

*/

            query = "on_Physical Top Bottom 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);
/*
            query = "in_ContGeneric Inner_Object Outer_Object 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);
*/

            //attribute
            query = "empty_hand Hand 0 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "full_hand Hand 0 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "closed_hand Hand 0 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "opened_hand Hand 0 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "graspedBy Hand Target 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);


/*
            query = "opened_door Door 0 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

            query = "closed_door Door 0 0 0 100 FSD";
            setRequset(request, query);
            FSDRequestor.publish(request);
            log.info(String.format(query));
            Thread.sleep(100);

*/


      } catch (Exception e) {
        e.printStackTrace();
      }

    }

    public void setRequset(MonitorServiceRequest request, String query){
        request.setPredicate(query.split(" ")[0]);
        request.setParam1(query.split(" ")[1]);
        request.setParam2(query.split(" ")[2]);
        request.setParam3(query.split(" ")[3]);
        request.setParam4(query.split(" ")[4]);
        request.setStatus(Integer.parseInt(query.split(" ")[5]));
        request.setManager(query.split(" ")[6]);
    }

    public void sleep(int n) {
        try {
            Thread.sleep(n);
        } catch (Exception e) {
        }
    }

}

