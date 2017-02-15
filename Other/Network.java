import java.util.Arrays;
import org.neuroph.core.NeuralNetwork;
import org.neuroph.nnet.MultiLayerPerceptron;
import org.neuroph.nnet.learning.BackPropagation;
import org.neuroph.core.data.DataSet;
import org.neuroph.core.data.DataSetRow;
import org.neuroph.util.TransferFunctionType;

public class Network {
	public static void main(String[] args) {
		DataSet trainingSet = DataSet.createFromFile("/Users/hprobotics/Desktop/Center Vortex/pixels.csv", 2500, 2, ",");
		System.out.println(trainingSet.getRows());

		MultiLayerPerceptron myMlPerceptron = new MultiLayerPerceptron(TransferFunctionType.SIGMOID, 2500, 3, 2);
		BackPropagation backprop = new BackPropagation();
				
		//Network Setup:
		backprop.setMaxError(.001);
		backprop.setLearningRate(.001);
		backprop.setMaxIterations(10000);
		
		//myMlPerceptron.learn(trainingSet, backprop);
		//testNeuralNetwork(myMlPerceptron, trainingSet);
	}

	public static void testNeuralNetwork(NeuralNetwork<BackPropagation> nnet, DataSet testSet) {
		for(DataSetRow dataRow: testSet.getRows()) {
			nnet.setInput(dataRow.getInput());
			nnet.calculate();
			double[] networkOutput = nnet.getOutput();
			
			System.out.print("Input: " + Arrays.toString(dataRow.getInput()));
			System.out.println(" Output: " + Arrays.toString(networkOutput));
		}
	}

}