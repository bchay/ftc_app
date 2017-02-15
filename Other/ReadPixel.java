import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.imageio.ImageIO;

public class ReadPixel {
	public static void main(String[] args) {
		BufferedImage img = null;
		ArrayList<String> photoPaths = new ArrayList<>();
		HashMap<String, List<Integer>> pixelList = new HashMap<String, List<Integer>>();
		HashMap<String, List<Integer>> outputs = new HashMap<String, List<Integer>>();
		FileWriter fileWriter = null;
		String regex = "\\.(jpg)";
		Pattern pattern = Pattern.compile(regex);
		
		try {
			Files.walk(Paths.get("/Users/hprobotics/Desktop/Center Vortex")).filter(Files::isRegularFile).forEach(str -> {
			    Matcher m = pattern.matcher(str.toString());
			    if(m.find()) {
			    	photoPaths.add(str.toString());
			    	//System.out.println(str);
			    }
			});
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		for(String path : photoPaths) {
			try {
				img = ImageIO.read(new File(path));
			} catch (IOException e) {
				e.printStackTrace();
			}
			
			ArrayList<Integer> grayImage = new ArrayList<>();
			for(int x = 0; x < img.getWidth(); x++) {
				for(int y = 0; y < img.getHeight(); y++) {
					int rgb = img.getRGB(x, y);
					int r = (rgb >> 16) & 0xFF;
					int g = (rgb >> 8) & 0xFF;
					int b = (rgb & 0xFF);
					int gray = (r + g + b) / 3;
					grayImage.add(gray);
				}
			}
			pixelList.put(path, grayImage);
		}
		
		try(BufferedReader br = new BufferedReader(new FileReader("/Users/hprobotics/Desktop/Center Vortex/output.txt"))) {
		    StringBuilder sb = new StringBuilder();
		    String line = br.readLine();

		    while (line != null) {
		    	final String l = line;
		    	outputs.put(l.substring(0, l.indexOf(", ")), new ArrayList<Integer>() {{
		    		add(Integer.valueOf(l.substring(l.indexOf(", ") + 2).substring(0, l.substring(l.indexOf(", ") + 2).indexOf(","))));
		    	    add(Integer.valueOf(l.substring(l.indexOf(", ") + 2).substring(l.substring(l.indexOf(", ") + 2).indexOf(",") + 2)));
		    	}});
		    	
		        sb.append(line);
		        sb.append(System.lineSeparator());
		        line = br.readLine();
		    }
			
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		} catch (IOException e1) {
			e1.printStackTrace();
		}
		
		/*
		for (Map.Entry<String, List<Integer>> entry : pixelList.entrySet()) {
		    System.out.println("Pixel List - " + entry.getKey() + ": " + entry.getValue());
		}
		*/

        try {
			fileWriter = new FileWriter("/Users/hprobotics/Desktop/Center Vortex/pixels.csv");
		} catch (IOException e) {
			e.printStackTrace();
		}
        
        Iterator it = pixelList.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<String, List<Integer>> pair = (Map.Entry<String, List<Integer>>) it.next();
            
            try {
				fileWriter.append(String.valueOf(pair.getValue()).toString().replace("[", "").replace("]", "").replace(" ", "").trim() + "," + outputs.get(pair.getKey()).get(0) + "," + outputs.get(pair.getKey()).get(1) + "\n");
			} catch (IOException e) {
				e.printStackTrace();
			}
            it.remove();
        }
        
        try {
			fileWriter.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}