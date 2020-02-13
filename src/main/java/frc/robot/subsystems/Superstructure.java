/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

import frc.lib.util.DriveSignal;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.states.TimedLEDState;
import frc.robot.subsystems.Hopper.HopperControlState;
import frc.robot.subsystems.Intake.IntakeControlState;
import frc.robot.subsystems.requests.Request;
import frc.robot.subsystems.requests.RequestList;

/**
 * Add your docs here.
 */
public class Superstructure extends Subsystem {

    private static Superstructure mInstance = null;

    public static Superstructure getInstance() {
        if(mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    private Drive mDrive = Drive.getInstance();
    private Shooter mShooter = Shooter.getInstance();
    private Spinner mSpinner = Spinner.getInstance();
    private Hopper mHopper = Hopper.getInstance();
    private Intake mIntake = Intake.getInstance();

    private RequestList activeRequests;
    private ArrayList<RequestList> queuedRequests;
    private Request currentRequest;

    private boolean newRequests = false;
    private boolean activeRequestsCompleted = false;
    private boolean allRequestsCompleted = false;

    private Superstructure() {}

    
    public boolean requestsCompleted() {
        return allRequestsCompleted;
    }

    private void setQueuedRequests(RequestList requests) {
        queuedRequests.clear();
        queuedRequests.add(requests);
    }

    private void setQueuedRequests(List<RequestList> requests) {
        queuedRequests.clear();
        queuedRequests = new ArrayList<>(requests.size());
        for (RequestList list : requests) {
            queuedRequests.add(list);
        }
    }

    private void setActiveRequests(RequestList requests) {
        activeRequests = requests;
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void request(Request r) {
        setActiveRequests(new RequestList(Arrays.asList(r), false));
        setQueuedRequests(new RequestList());
    }

    public void request(Request active, Request queue) {
        setActiveRequests(new RequestList(Arrays.asList(active), false));
        setQueuedRequests(new RequestList(Arrays.asList(queue), false));
    }

    public void request(RequestList requestList) {
        setActiveRequests(requestList);
        setQueuedRequests(new RequestList());
    }

    public void request(RequestList activeList, RequestList queuedList) {
        setActiveRequests(activeList);
        setQueuedRequests(queuedList);
    }

    public void addActiveRequest(Request request) {
        activeRequests.add(request);
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void addFormostActiveRequest(Request request) {
        activeRequests.addToForeFront(request);
        newRequests = true;
        activeRequestsCompleted = false;
        allRequestsCompleted = false;
    }

    public void queue(Request request) {
        queuedRequests.add(new RequestList(Arrays.asList(request), false));
    }

    public void queue(RequestList list) {
        queuedRequests.add(list);
    }

    public void replaceQueue(Request request) {
        setQueuedRequests(new RequestList(Arrays.asList(request), false));
    }

    public void replaceQueue(RequestList list) {
        setQueuedRequests(list);
    }

    public void replaceQueue(List<RequestList> lists) {
        setQueuedRequests(lists);
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
        
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    if (!activeRequestsCompleted) {
                        if (newRequests) {
                            if (activeRequests.isParallel()) {
                                boolean allActivated = true;
                                for (Iterator<Request> iterator = activeRequests.getRequests().iterator(); iterator
                                        .hasNext();) {
                                    Request request = iterator.next();
                                    boolean allowed = request.allowed();
                                    allActivated &= allowed;
                                    if (allowed)
                                        request.act();
                                }
                                newRequests = !allActivated;
                            } else {
                                if (activeRequests.isEmpty()) {
                                    activeRequestsCompleted = true;
                                    return;
                                }
                                currentRequest = activeRequests.remove();
                                currentRequest.act();
                                newRequests = false;
                            }
                        }
                        if (activeRequests.isParallel()) {
                            boolean done = true;
                            for (Request request : activeRequests.getRequests()) {
                                done &= request.isFinished();
                            }
                            activeRequestsCompleted = done;
                        } else if (currentRequest.isFinished()) {
                            if (activeRequests.isEmpty()) {
                                activeRequestsCompleted = true;
                            } else if (activeRequests.getRequests().get(0).allowed()) {
                                newRequests = true;
                                activeRequestsCompleted = false;
                            }
                        }
                    } else {
                        if (!queuedRequests.isEmpty()) {
                            setActiveRequests(queuedRequests.remove(0));
                        } else {
                            allRequestsCompleted = true;
                        }
                    }
    
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();

            }

        });
    }


    public void autoPositionControl() {
        
    }


    public void autoRotationControl() {

    }


    public Request driveUntilControlPanelRequest() {
        Request request = new Request(){
        
            @Override
            public void act() {
                mDrive.setOpenLoop(new DriveSignal(0.3, 0.3));
            }

            @Override
            public boolean isFinished() {
                return mSpinner.seesControlPanel();
            }
        };

        request.withPrerequisite(mSpinner.deployedPrerequisite);

        return request;
    }





    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

	public boolean isAtDesiredState() {
		return false;
	}


}
