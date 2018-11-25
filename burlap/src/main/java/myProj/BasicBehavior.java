package myProj;
import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.valuefunction.QFunction;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;

import java.awt.*;
import java.util.List;

public class BasicBehavior {

	GridWorldDomain gwdg;
	OOSADomain domain;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	HashableStateFactory hashingFactory;
	SimulatedEnvironment env;
	GoalBasedRF rf;

	int MAP_SIZE=9;
	protected int [][] map = new int[][]{
			{0,0,0,1,0,1,0,1,0},
			{1,1,0,1,0,1,0,1,0},
			{0,1,0,0,0,0,0,0,0},
			{0,1,1,1,0,1,0,1,1},
			{0,0,0,0,0,1,0,1,0},
			{1,1,1,1,0,1,0,1,0},
			{0,1,0,0,0,1,0,0,0},
			{0,1,0,1,1,1,1,1,0},
			{0,0,0,0,0,0,0,1,0}};
	protected int [][] map2 = new int[][]{
			{0,0,0,1,0,1,0,1,0},
			{1,1,0,1,0,1,0,1,0},
			{0,1,0,0,0,0,0,0,0},
			{0,1,1,1,0,1,0,1,1},
			{0,0,0,0,0,1,0,1,0},
			{1,1,1,1,0,1,0,1,0},
			{0,1,0,0,0,1,0,0,0},
			{0,1,0,1,1,1,1,1,0},
			{0,0,0,0,0,0,0,1,0}};
			public BasicBehavior(){
				gwdg = new GridWorldDomain(map);
				gwdg.setProbSucceedTransitionDynamics(0.8);
				//gwdg.setMapToFourRooms();
				tf = new GridWorldTerminalFunction((MAP_SIZE-1), (MAP_SIZE-1));
				gwdg.setTf(tf);
				goalCondition = new TFGoalCondition(tf);
				domain = gwdg.generateDomain();
		
				initialState = new GridWorldState(new GridAgent(0, 0), new GridLocation((MAP_SIZE-1), (MAP_SIZE-1), "loc0"));
				hashingFactory = new SimpleHashableStateFactory();
		
				env = new SimulatedEnvironment(domain, initialState);
				this.rf=new GoalBasedRF(this.goalCondition, 100, -1);
				//GridWorldState rState = new GridWorldState(MAP_SIZE-1,MAP_SIZE-2, new GridLocation((MAP_SIZE-1), (MAP_SIZE-1), "loc0"));
				//this.rf.setReward(rState,-100);
				((FactoredModel)domain.getModel()).setRf(this.rf);
		
				//VisualActionObserver observer = new VisualActionObserver(domain, 
				//	GridWorldVisualizer.getVisualizer(gwdg.getMap()));
				//observer.initGUI();
				//env.addObservers(observer);
			}
		
		
			public void visualize(String outputpath){
				Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
				new EpisodeSequenceVisualizer(v, domain, outputpath);
			}
		
			
		
			public void valueIterationExample(String outputPath,int numIter){
		
				for(int i=1; i<numIter+1;i++){
					long tStart = System.currentTimeMillis();

					ValueIteration planner = new ValueIteration(domain, 0.5, hashingFactory, 0.001, i);
					Policy p = planner.planFromState(initialState);
					Episode ea=PolicyUtils.rollout(p, initialState,domain.getModel(),i);
					long tEnd = System.currentTimeMillis();
					long tDelta = tEnd - tStart;
					double elapsedSeconds = tDelta / 1000.0;
					if (i==1||i==5||i%100==0)
						manualValueFunctionVis((ValueFunction)planner, p);
					System.out.println("vi " + i + ": " + ((ValueFunction)planner).value(initialState) + " time: "+elapsedSeconds+" delta : "+planner._delta);
					
				}
				//PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "vi");
				// Planner planner = new ValueIteration(domain, 0.99, hashingFactory, 0.001, numIter);
				// Policy p = planner.planFromState(initialState);
				// simpleValueFunctionVis((ValueFunction)planner, p);
				// PolicyUtils.rollout(p, initialState, domain.getModel(),numIter).write(outputPath + "vi");
				//manualValueFunctionVis((ValueFunction)planner, p);
		
			}
			public void policyIterationExample(String outputPath,int numIter){
				
				for(int i=numIter; i<numIter+1;i++){
					long tStart = System.currentTimeMillis();
					PolicyIteration planner = new PolicyIteration(domain, 0.3, hashingFactory, 0.001,i, i);
					Policy p = planner.planFromState(initialState);
					long tEnd = System.currentTimeMillis();
					long tDelta = tEnd - tStart;
					double elapsedSeconds = tDelta / 1000.0;
					Episode ea=PolicyUtils.rollout(p, initialState,domain.getModel(),i);
					System.out.println("pi " + i + ": " + ((ValueFunction)planner).value(initialState) + " time: "+elapsedSeconds+" delta : "+planner._delta);
					if (i==1||i==5||i%10==0)
						manualValueFunctionVis((ValueFunction)planner, p);

				}
				// Planner planner = new PolicyIteration(domain, 0.99, hashingFactory, 0.001,numIter, numIter);
				// Policy p = planner.planFromState(initialState);
				// manualValueFunctionVis((ValueFunction)planner, p);
				
		
				//simpleValueFunctionVis((ValueFunction)planner, p);
		
			}
		
			public void qLearningExample(String outputPath,int numIter){
		
		
				QLearning agent = new QLearning(domain, 0.99, hashingFactory, 0., 0.99);
				long tStart = System.currentTimeMillis();
				//run learning for 50 episodes
				for(int i = 1; i < numIter+1; i++){
					Episode e = agent.runLearningEpisode(env);
					long tEnd = System.currentTimeMillis();
					long tDelta = tEnd - tStart;
					double elapsedSeconds = tDelta / 1000.0;
					
					System.out.println(agent._value(initialState) + " "+elapsedSeconds);
					if (i==1000){
						Policy p=agent.learningPolicy;
						simpleValueFunctionVis((ValueFunction)agent, p);
					}
					//reset environment for next learning episode
					env.resetEnvironment();
				}
				//Policy p = agent.planFromState(initialState);
				//PolicyUtils.rollout(p, initialState,domain.getModel(),numIter).write(outputPath + "ql");
				//manualValueFunctionVis((ValueFunction)agent, p);
		
			}
		
		
			public void sarsaLearningExample(String outputPath,int numIter){
		
				SarsaLam agent = new SarsaLam(domain, 0.99, hashingFactory, 0.99, 0.1, 1);
		
				//run learning for 50 episodes
				for(int i = 0; i < numIter; i++){
					Episode e = agent.runLearningEpisode(env);
		
					System.out.println("sl "+i + ": " + ((ValueFunction)agent).value(initialState));
		
					//reset environment for next learning episode
					env.resetEnvironment();
				}
				Policy p = agent.planFromState(initialState);
				PolicyUtils.rollout(p, initialState,domain.getModel(),numIter).write(outputPath + "sl");
				manualValueFunctionVis((ValueFunction)agent, p);
			}
		
			public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p){
		
				List<State> allStates = StateReachability.getReachableStates(
					initialState, domain, hashingFactory);
				ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
					allStates, (MAP_SIZE), (MAP_SIZE), valueFunction, p);
				gui.initGUI();
		
			}
		
			public void manualValueFunctionVis(ValueFunction valueFunction, Policy p){
		
				List<State> allStates = StateReachability.getReachableStates(
					initialState, domain, hashingFactory);
		
		
				//define color function
				LandmarkColorBlendInterpolation rb = new LandmarkColorBlendInterpolation();
				rb.addNextLandMark(0., Color.RED);
				rb.addNextLandMark(1., Color.BLUE);
		
				//define a 2D painter of state values, 
				//specifying which attributes correspond to the x and y coordinates of the canvas
				StateValuePainter2D svp = new StateValuePainter2D(rb);
				svp.setXYKeys("agent:x", "agent:y", 
					new VariableDomain(0, (MAP_SIZE)), new VariableDomain(0, (MAP_SIZE)), 
					1, 1);
		
				//create our ValueFunctionVisualizer that paints for all states
				//using the ValueFunction source and the state value painter we defined
				ValueFunctionVisualizerGUI gui = new ValueFunctionVisualizerGUI(
					allStates, svp, valueFunction);
		
				//define a policy painter that uses arrow glyphs for each of the grid world actions
				PolicyGlyphPainter2D spp = new PolicyGlyphPainter2D();
				spp.setXYKeys("agent:x", "agent:y", new VariableDomain(0, (MAP_SIZE)), 
					new VariableDomain(0, (MAP_SIZE)), 
					1, 1);
				spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_NORTH, new ArrowActionGlyph(0));
				spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_SOUTH, new ArrowActionGlyph(1));
				spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_EAST, new ArrowActionGlyph(2));
				spp.setActionNameGlyphPainter(GridWorldDomain.ACTION_WEST, new ArrowActionGlyph(3));
				spp.setRenderStyle(PolicyGlyphPainter2D.PolicyGlyphRenderStyle.DISTSCALED);
		
		
				//add our policy renderer to it
				gui.setSpp(spp);
				gui.setPolicy(p);
		
				//set the background color for places where states are not rendered to grey
				gui.setBgColor(Color.GRAY);
		
				//start it
				gui.initGUI();
		
		
		
			}
		
		
			public void experimentAndPlotter(){
		
				//different reward function for more structured performance plots
				
		
				/**
				 * Create factories for Q-learning agent and SARSA agent to compare
				 */
				LearningAgentFactory qLearningFactory = new LearningAgentFactory() {
		
					public String getAgentName() {
						return "Q-Learning";
					}
		
		
					public LearningAgent generateAgent() {
						return new QLearning(domain, 0.99, hashingFactory, 0.3, 0.99);
					}
				};
		
				LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {
		
					public String getAgentName() {
						return "SARSA";
					}
		
		
					public LearningAgent generateAgent() {
						return new SarsaLam(domain, 0.99, hashingFactory, 0.0, 0.99, 1.);
					}
				};
				// LearningAgentFactory policyLearningFactory = new LearningAgentFactory() {
		
				// 	public String getAgentName() {
				// 		return "Policy";
				// 	}
		
		
				// 	public LearningAgent generateAgent() {
				// 		return new ValueIteration(domain, 0.99, hashingFactory, 0.001, 5);
				// 	}
				// };
				
				LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(
					env, 10, 300, qLearningFactory, sarsaLearningFactory);
				exp.setUpPlottingConfiguration(500, 250, 2, 1000,
						TrialMode.MOST_RECENT_AND_AVERAGE,
						PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
						PerformanceMetric.AVERAGE_EPISODE_REWARD);
		
				exp.startExperiment();
				exp.writeStepAndEpisodeDataToCSV("expData");
		
			}
		
		
			public static void main(String[] args) {
		
				BasicBehavior example = new BasicBehavior();
				String outputPath = "output/";
		
				//example.BFSExample(outputPath);gen
				//example.DFSExample(outputPath);
				//example.AStarExample(outputPath);
				//example.valueIterationExample(outputPath,200);
				//example.policyIterationExample(outputPath,100);
				example.qLearningExample(outputPath,1000);
				//example.sarsaLearningExample(outputPath,300);
		
				//example.experimentAndPlotter();
		
				//example.visualize(outputPath);
		
			}
		
		}
		
