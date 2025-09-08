package com.example.ccn.utils;


import com.google.gson.*;

class Node {
    double x;
    double y;
}

public class NodeParser {
    public static void parse(JsonObject json) {

        // Parse the JSON
        JsonObject jsonObject = JsonParser.parseString(json.toString()).getAsJsonObject();

        // Get the 'smooth_path' array
        JsonArray smoothPathArray = jsonObject.getAsJsonArray("smooth_path");

        // Iterate over it
        for (JsonElement element : smoothPathArray) {
            JsonObject nodeObj = element.getAsJsonObject();
            double x = nodeObj.get("x").getAsDouble();
            double y = nodeObj.get("y").getAsDouble();
            System.out.println("Node -> x: " + x + ", y: " + y);
        }

        // OR: Deserialize directly into List<Node>
        Gson gson = new Gson();
        Node[] nodeArray = gson.fromJson(smoothPathArray, Node[].class);

        System.out.println("Deserialized Nodes:");
        for (Node n : nodeArray) {
            System.out.println("x = " + n.x + ", y = " + n.y);
        }
    }
}