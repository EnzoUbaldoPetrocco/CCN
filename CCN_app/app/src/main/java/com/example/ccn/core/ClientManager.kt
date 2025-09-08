package com.example.ccn.core

import android.content.Context
import android.content.SharedPreferences
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.util.Base64
import android.util.Log
import com.aldebaran.qi.Future
import com.aldebaran.qi.sdk.QiContext
import com.aldebaran.qi.sdk.`object`.actuation.ExplorationMap
import com.aldebaran.qi.sdk.`object`.geometry.Transform
import com.aldebaran.qi.sdk.`object`.image.TimestampedImageHandle
import com.aldebaran.qi.sdk.`object`.streamablebuffer.StreamableBuffer
import com.aldebaran.qi.sdk.`object`.streamablebuffer.StreamableBufferFactory
import com.aldebaran.qi.sdk.builder.ExplorationMapBuilder
import com.aldebaran.qi.sdk.builder.TakePictureBuilder
import com.example.ccn.R
import okhttp3.*
import okhttp3.MediaType.Companion.toMediaType
import okhttp3.MediaType.Companion.toMediaTypeOrNull
import okhttp3.RequestBody.Companion.toRequestBody
import org.json.JSONArray
import org.json.JSONException
import org.json.JSONObject
import java.io.IOException
import java.nio.ByteBuffer
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit


/*
    Manager tha provides and saves the [ExplorationMap]
 */
object ClientManager {

    private const val TAG = "ClientManager"
    private const val MAP_FILENAME = "map.txt"

    val client = OkHttpClient()


    // The cached map
    private var explorationMap: ExplorationMap? = null

    /**
     * Get robot position according to the server
     */
    @JvmStatic
    fun getRobotPosition(qiContext: QiContext): JSONObject {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl = "$serverUrlInit/robot/position"

        val request = Request.Builder()
            .url(serverUrl)
            .build()

        var json: JSONObject = JSONObject()


        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                // Handle error
                Log.d(TAG, "Error ${e.message}")
            }

            override fun onResponse(call: Call, response: Response){
                if (response.isSuccessful) {
                    json = response.body?.string()?.let { JSONObject(it) }!!

                } else {
                    // Handle non-200 responses
                    Log.d(TAG, "Error with code ${response.code}")
                }
            }
        })
        Log.d(TAG, "Robot position from server is: $json")
        return json
    }

    /**
     * Get robot position according to the server
     */
    @JvmStatic
    fun getGoalPosition(qiContext: QiContext): JSONObject {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl = "$serverUrlInit/goal/position"

        val request = Request.Builder()
            .url(serverUrl)
            .build()

        var json: JSONObject = JSONObject()


        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                // Handle error
                Log.d(TAG, "Error ${e.message}")
            }

            override fun onResponse(call: Call, response: Response){
                if (response.isSuccessful) {
                    json = response.body?.string()?.let { JSONObject(it) }!!

                } else {
                    // Handle non-200 responses
                    Log.d(TAG, "Error with code ${response.code}")
                }
            }
        })
        Log.d(TAG, "Goal position from server is: $json")
        return json
    }

    /**
     * post localization x,y, and wz of the robot
     *
     * @param qiContext the qi context
     */
    @JvmStatic
    fun postRobotPosition(qiContext: QiContext) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl: String = serverUrlInit + "/robot/position"
        Log.d(TAG, "Posting to url: $serverUrl")

        // Get the robot frame
        val actuation = qiContext.actuation
        val mapping = qiContext.mapping
        val jsonMediaType = "application/json; charset=utf-8".toMediaType()

        actuation.async().robotFrame()
            .andThenCompose { robotFrame ->
                mapping.async().mapFrame()
                    .andThenCompose { mapFrame ->
                        robotFrame.async().computeTransform(mapFrame)
                    }
            }
            .thenConsume { futureTransform ->
                if (futureTransform.isSuccess) {
                    val transformTime = futureTransform.get()
                    val x = transformTime.transform.translation.x
                    val y = transformTime.transform.translation.y
                    val theta = transformTime.transform.rotation.z

                    Log.d(TAG, "Transform received: x=$x, y=$y, theta=$theta")

                    val jsonBody = JSONObject().apply {
                        put("x", x)
                        put("y", y)
                        put("theta", theta)
                    }.toString()

                    Log.d(TAG, "Posting robot position to server: $jsonBody")

                    val requestBody = jsonBody.toRequestBody(jsonMediaType)
                    val request = Request.Builder()
                        .url(serverUrl)
                        .post(requestBody)
                        .addHeader("Content-Type", "application/json")
                        .build()

                    client.newCall(request).enqueue(object : Callback {
                        override fun onFailure(call: Call, e: IOException) {
                            Log.e(TAG, "Failed to upload robot position: ${e.message}", e)
                        }

                        override fun onResponse(call: Call, response: Response) {
                            if (response.isSuccessful) {
                                Log.i(TAG, "Robot position uploaded successfully")
                            } else {
                                Log.e(TAG, "Server error: ${response.code}")
                            }
                        }
                    })
                } else {
                    Log.e(TAG, "Failed to compute transform", futureTransform.error)
                }
            }

    }

    /**
     * post localization x,y, and wz of the robot
     *
     * @param qiContext the qi context
     */
    @JvmStatic
    fun postRobotPosition(qiContext: QiContext, transform: Transform) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl: String = serverUrlInit + "/robot/position"
        Log.d(TAG, "Posting to url: $serverUrl")

        val jsonMediaType = "application/json; charset=utf-8".toMediaType()
        // Build JSON payload with map metadata
        val jsonBody = JSONObject().apply {
            put("x", transform.translation.x)
            put("y", transform.translation.y)
            put("theta", transform.rotation.z)
        }.toString()

        Log.d(TAG, "Posting robot position to server: $jsonBody")

        val requestBody = jsonBody.toRequestBody(jsonMediaType)

        val request = Request.Builder()
            .url(serverUrl)
            .post(requestBody)
            .addHeader("Content-Type", "application/json")
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                Log.e(TAG, "Failed to upload robot position: ${e.message}", e)
            }

            override fun onResponse(call: Call, response: Response) {
                if (response.isSuccessful) {
                    Log.i(TAG, "robot position uploaded successfully")
                } else {
                    Log.e(TAG, "Server error: ${response.code}")
                }
            }
        })

    }



    /**
     * post localization x,y, and wz of the robot
     *
     * @param qiContext the qi context
     */
    @JvmStatic
    fun postGoalPosition(qiContext: QiContext, positionJson: JSONObject) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl: String = serverUrlInit + "/goal/position"
        Log.d(TAG, "Posting to url: $serverUrl")

        val jsonMediaType = "application/json; charset=utf-8".toMediaType()

        // Build JSON payload with map metadata
        val jsonBody = JSONObject().apply {
            put("x", positionJson["x"])
            put("y", positionJson["y"])
            put("theta", positionJson["theta"])
        }.toString()

        Log.d(TAG, "Posting goal position to server: $jsonBody")


        val requestBody = jsonBody.toRequestBody(jsonMediaType)

        val request = Request.Builder()
            .url(serverUrl)
            .post(requestBody)
            .addHeader("Content-Type", "application/json")
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                Log.e(TAG, "Failed to upload Goal Position: ${e.message}", e)
            }

            override fun onResponse(call: Call, response: Response) {
                if (response.isSuccessful) {
                    Log.i(TAG, "Goal position uploaded successfully")
                } else {
                    Log.e(TAG, "Server error: ${response.code}")
                }
            }
        })
    }


    /**
    Send the specified map to my server

    @param context the context
    @param map the map to save
    @param serverUrl the url of the server
    @return A [Future] wrapping the operation
     */
    @JvmStatic
    fun sendMapToServer(context: Context, map: ExplorationMap): Future<Void> {
        // Store for reuse
        if (map == null){
            Log.d(TAG, "Map was null")
        }
        this.explorationMap = map
        val prefs: SharedPreferences =
            context.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", context.getString(R.string.serverUrl))
        val serverUrl: String = serverUrlInit+ "/map-upload";
        Log.d(TAG, "Posting to url: $serverUrl");

        // Extract graphical representation
        val mapTop = map.topGraphicalRepresentation
        val imageBuffer = mapTop.image.data.asReadOnlyBuffer()
        val imageBytes = ByteArray(imageBuffer.remaining())
        imageBuffer.get(imageBytes)
        val encodedImage = Base64.encodeToString(imageBytes, Base64.NO_WRAP)

        // Build JSON payload with map metadata
        val jsonMetadata = JSONObject().apply {
            put("x", mapTop.x)
            put("y", mapTop.y)
            put("theta", mapTop.theta)
            put("scale", mapTop.scale)
            put("image_base64", encodedImage)
        }

        // Serialize and upload
        return map.async().serializeAsStreamableBuffer()
            .andThenConsume { streamableBuffer: StreamableBuffer ->
                Log.d(TAG, "Map serialized successfully")

                val size = streamableBuffer.size
                require(size <= Int.MAX_VALUE) { "Buffer size too large to fit in a byte array" }

                // Read the entire buffer from offset 0
                val byteBuffer = streamableBuffer.read(0, size)

                // Convert ByteBuffer to ByteArray
                val mapBytes = ByteArray(byteBuffer.remaining())
                byteBuffer.get(mapBytes)

                // Now send to server
                uploadMapToServer(serverUrl, jsonMetadata.toString(), mapBytes)
                Log.d(TAG, "Posted map position to server")
            }
    }

    /**
     * helper function of uploading the map to the server
     *
     * @param url the url of the server
     * @param jsonMetadata the object containing x,y,theta,scale and image_base64 of the map
     * @param mapBytes byteArray of the map
     */
    private fun uploadMapToServer(
        url: String,
        jsonMetadata: String,
        mapBytes: ByteArray
    ) {

        val requestBody = MultipartBody.Builder()
            .setType(MultipartBody.FORM)
            .addFormDataPart("metadata", null, jsonMetadata.toRequestBody("application/json".toMediaType()))
            .addFormDataPart("map_file", "map.bin",
                mapBytes.toRequestBody("application/octet-stream".toMediaType())
            )
            .build()

        val request = Request.Builder()
            .url(url)
            .post(requestBody)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                Log.e(TAG, "Failed to upload map: ${e.message}", e)
            }

            override fun onResponse(call: Call, response: Response) {
                if (response.isSuccessful) {
                    Log.i(TAG, "Map uploaded successfully")
                } else {
                    Log.e(TAG, "Server error: ${response.code}")
                }
            }
        })
    }

    /**
     * Provide the map.
     *
     * @param qiContext the qiContext
     * @param url url of the server
     * @return A [Future] wrapping the operation
     */
    @JvmStatic
    fun loadMapFromServer(qiContext: QiContext) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val url: String = serverUrlInit + "/map-download"

        val request = Request.Builder()
            .url(url)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                Log.e("MapLoad", "Failed to download map: ${e.message}")
            }

            override fun onResponse(call: Call, response: Response) {
                if (!response.isSuccessful || response.body == null) {
                    Log.e("MapLoad", "Unsuccessful response from server")
                    return
                }

                val mapBytes = response.body!!.bytes()

                val streamableBuffer = StreamableBufferFactory.fromBytes(mapBytes)


                explorationMap = ExplorationMapBuilder.with(qiContext)
                    .withStreamableBuffer(streamableBuffer)
                    .build()

                //Cache the map
                //this.explorationMap = explorationMap

            }
        })

        this.explorationMap = explorationMap
        Log.d(TAG, "Got map position from server")
    }

    /**
     * generetes StreamableBuffer from Bytes
     *
     * @param data ByteArray to be transformed
     * @return A [StreamableBuffer] object containing the information contained in data.
     */
    fun StreamableBufferFactory.fromBytes(data: ByteArray): StreamableBuffer {
        return fromFunction(data.size.toLong()) { offset, size ->
            val buffer = ByteArray(size.toInt())
            System.arraycopy(data, offset.toInt(), buffer, 0, size.toInt())
            ByteBuffer.wrap(buffer)
        }
    }

    @JvmStatic
    fun getNavigationPath(qiContext: QiContext, back: String = "false"): JSONObject {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl = serverUrlInit + "/navigation/path"

        val request = Request.Builder()
            .url(serverUrl)
            .addHeader("back", back)
            .build()

        val latch = CountDownLatch(1)
        var json_obj = JSONObject()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                Log.d(TAG, "Error ${e.message}")
                latch.countDown()
            }

            override fun onResponse(call: Call, response: Response) {
                try {
                    if (response.isSuccessful) {
                        val body = response.body?.string()
                        json_obj = JSONObject(body ?: "{}")
                        Log.d(TAG, "json object is $json_obj")
                    } else {
                        Log.d(TAG, "Error with code ${response.code}")
                    }
                } catch (e: Exception) {
                    Log.d(TAG, "Exception in response: ${e.message}")
                } finally {
                    latch.countDown()
                }
            }
        })

        latch.await() // Blocks until latch.countDown() is called
        Log.d(TAG, "Got navigation trajectory from server: ${json_obj.toString()}")
        return json_obj
    }

    /**
     * retrieves a point to navigate to
     * @param qiContext the qi context
     * @return a [JSONObject] containing the x, y, theta
     */
    @JvmStatic
    fun getNavigationPoint(qiContext: QiContext): JSONObject{
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl = serverUrlInit + "/navigation/go_to"

        val request = Request.Builder()
            .url(serverUrl)
            .build()

        var json: JSONObject = JSONObject()


        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                // Handle error
                Log.d(TAG, "Error ${e.message}")
            }

            override fun onResponse(call: Call, response: Response){
                if (response.isSuccessful) {
                    json = response.body?.string()?.let { JSONObject(it) }!!

                } else {
                    // Handle non-200 responses
                    Log.d(TAG, "Error with code ${response.code}")
                }
            }
        })
        Log.d(TAG, "Navigation point from server: $json")
        return json
    }


    /**
     * retrieves the navigation path
     * @param qiContext the qi context
     * @return a [JSONObject] containing the nodes, the optimal path, the smooth path, figure (ImageBase64) and figure_format (png).
     */
    @JvmStatic
    fun getNavigationPathWithPlots(qiContext: QiContext): JSONObject{
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl = serverUrlInit + "/navigation/path"

        val request = Request.Builder()
            .url(serverUrl)
            .build()

        var json: JSONObject = JSONObject()


        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                // Handle error
                Log.d(TAG, "Error ${e.message}")
            }

            override fun onResponse(call: Call, response: Response){
                if (response.isSuccessful) {
                    json = response.body?.string()?.let { JSONObject(it) }!!

                } else {
                    // Handle non-200 responses
                    Log.d(TAG, "Error with code ${response.code}")
                }
            }
        })
        Log.d(TAG, "Got navigation trajectory with plot from server: ${json.toString()}")
        return json
    }


    /**
     * visualize robot position in map
     */
    @JvmStatic
    fun getRobotInMap(qiContext: QiContext): Bitmap? {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl = serverUrlInit + "/visualize/robot_position"

        val request = Request.Builder()
            .url(serverUrl)
            .build()

        var bitmap : Bitmap? = null

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                // Handle error
                Log.d(TAG, "Error ${e.message}")
            }

            override fun onResponse(call: Call, response: Response){
                if (response.isSuccessful) {
                    val inputStream = response.body?.byteStream()
                     bitmap = BitmapFactory.decodeStream(inputStream)

                    // Code to display this image in case I would like to:
                    //runOnUiThread {
                    //    imageView.setImageBitmap(bitmap)
                    //}
                } else {
                    // Handle non-200 responses
                    Log.d(TAG, "Error with code ${response.code}")
                }
            }
        })
        Log.d(TAG, "Got robot position in map")
        return bitmap
    }

    /**
     * sends the information contained in the camera
     */
    @JvmStatic
    fun postCameraInfo(qiContext: QiContext){
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl: String = serverUrlInit + "/camera"
        Log.d(TAG, "Posting to url: $serverUrl")

        val jsonMediaType = "application/json; charset=utf-8".toMediaType()

        Log.d(TAG, "before building the action")

        // Build the action.
        TakePictureBuilder.with(qiContext).buildAsync()
            .thenConsume { future ->
                if (future.hasError()) {
                    Log.e(TAG, "Could not build TakePicture", future.error)
                    return@thenConsume
                }

                val takePicture = future.get()
                // Run the action synchronously.
                val result: TimestampedImageHandle = takePicture.run()

                Log.d(TAG, "Picture taken")

                // 1. get a proxy to access the data
                val imageBuffer = result.image.value.data.asReadOnlyBuffer()
                // 2. copy the remote data value
                val imageBytes = ByteArray(imageBuffer.remaining())
                imageBuffer.get(imageBytes)
                val encodedImage = Base64.encodeToString(imageBytes, Base64.NO_WRAP)

                Log.d(TAG, "Picture encoded")

                // Build JSON payload with map metadata
                val jsonBody = JSONObject().apply {
                    put("image", encodedImage)
                    put("time", result.time)
                }.toString()


                val requestBody = jsonBody.toRequestBody(jsonMediaType)

                val request = Request.Builder()
                    .url(serverUrl)
                    .post(requestBody)
                    .addHeader("Content-Type", "application/json")
                    .build()

                client.newCall(request).enqueue(object : Callback {
                    override fun onFailure(call: Call, e: IOException) {
                        Log.e(TAG, "Failed to upload Camera Info: ${e.message}", e)
                    }
                    override fun onResponse(call: Call, response: Response) {
                        if (response.isSuccessful) {
                            Log.i(TAG, "Camera Information uploaded successfully")
                        } else {
                            Log.e(TAG, "Server error: ${response.code}")
                        }
                    }
                })
                Log.d(TAG, "Posted camera info to server")
            }

        Log.d(TAG, "take picture built")


    }

    /**
     * sends the information contained in the depth camera: TODO: understand how
     */
    @JvmStatic
    fun postDCameraInfo(qiContext: QiContext){
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl: String = serverUrlInit + "/depth_camera"
        Log.d(TAG, "Posting to url: $serverUrl")

        val jsonMediaType = "application/json; charset=utf-8".toMediaType()

        Log.d(TAG, "before building the action")

        // Build the action.
        TakePictureBuilder.with(qiContext).buildAsync()
            .thenConsume { future ->
                if (future.hasError()) {
                    Log.e(TAG, "Could not build TakePicture", future.error)
                    return@thenConsume
                }

                val takePicture = future.get()
                // Run the action synchronously.
                val result: TimestampedImageHandle = takePicture.run()

                Log.d(TAG, "Picture taken")

                // 1. get a proxy to access the data
                val imageBuffer = result.image.value.data.asReadOnlyBuffer()
                // 2. copy the remote data value
                val imageBytes = ByteArray(imageBuffer.remaining())
                imageBuffer.get(imageBytes)
                val encodedImage = Base64.encodeToString(imageBytes, Base64.NO_WRAP)

                Log.d(TAG, "Picture encoded")

                // Build JSON payload with map metadata
                val jsonBody = JSONObject().apply {
                    put("image", encodedImage)
                    put("time", result.time)
                }.toString()


                val requestBody = jsonBody.toRequestBody(jsonMediaType)

                val request = Request.Builder()
                    .url(serverUrl)
                    .post(requestBody)
                    .addHeader("Content-Type", "application/json")
                    .build()

                client.newCall(request).enqueue(object : Callback {
                    override fun onFailure(call: Call, e: IOException) {
                        Log.e(TAG, "Failed to upload Camera Info: ${e.message}", e)
                    }
                    override fun onResponse(call: Call, response: Response) {
                        if (response.isSuccessful) {
                            Log.i(TAG, "Camera Information uploaded successfully")
                        } else {
                            Log.e(TAG, "Server error: ${response.code}")
                        }
                    }
                })
                Log.d(TAG, "Posted camera info to server")
            }

        Log.d(TAG, "take picture built")
    }

    /**
     * Get robot localization based on data contained in the camera
     */
    @JvmStatic
    fun getRobotPositionCamera(qiContext: QiContext): JSONObject {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val serverUrlInit = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val serverUrl = serverUrlInit + "/robot/position_using_camera"
        Log.d(TAG, "Posting to url: $serverUrl")

        val jsonMediaType = "application/json; charset=utf-8".toMediaType()
        val resultJson = arrayOf(JSONObject()) // workaround for mutability in lambda
        val latch = CountDownLatch(1)

        val takePictureFuture = TakePictureBuilder.with(qiContext).buildAsync().get()

        val cameraTransform = qiContext.actuation.gazeFrame().computeTransform(qiContext.actuation.robotFrame())

        try {
            val result: TimestampedImageHandle = takePictureFuture.run()
            val imageBuffer = result.image.value.data.asReadOnlyBuffer()
            val imageBytes = ByteArray(imageBuffer.remaining())
            imageBuffer.get(imageBytes)
            val encodedImage = Base64.encodeToString(imageBytes, Base64.NO_WRAP)

            val jsonBody = JSONObject().apply {
                put("image", encodedImage)
                put("time", result.time)
                put("cameraTransform", cameraTransform.toString())
            }.toString()


            val requestBody = jsonBody.toRequestBody(jsonMediaType)
            val request = Request.Builder()
                .url(serverUrl)
                .post(requestBody)
                .addHeader("Content-Type", "application/json")
                .build()

            client.newCall(request).enqueue(object : Callback {
                override fun onFailure(call: Call, e: IOException) {
                    Log.e(TAG, "HTTP error: ${e.message}")
                    latch.countDown()
                }

                override fun onResponse(call: Call, response: Response) {
                    if (response.isSuccessful) {
                        val body = response.body?.string()
                        if (body != null) {
                            resultJson[0] = JSONObject(body)
                        }
                    } else {
                        Log.e(TAG, "Non-200 response: ${response.code}")
                    }
                    latch.countDown()
                }
            })

            val success = latch.await(5, TimeUnit.SECONDS)
            if (!success) {
                Log.e(TAG, "Timeout while waiting for server response")
            }


        } catch (e: Exception) {
            Log.e(TAG, "Error taking picture or sending request", e)
        }

        return resultJson[0]
    }


    /**
     * sends a text in the format of JSON: {user_input: [String]} to the server
     * to get the answer from the LLM
     * @param qiContext the qi Context
     * @param userInput a [String] containing the input from the user
     * @param a callback that returns a String contained in the json object
     *
     * @return nothing
     */
    @JvmStatic
    fun postRespond(qiContext: QiContext, userInput: String, callback: (String?) -> Unit) {
        val json = JSONObject().put("user_input", userInput)
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val body = RequestBody.create("application/json".toMediaTypeOrNull(), json.toString())
        val request = Request.Builder()
            .url("$baseUrl/respond")
            .post(body)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(null)
            override fun onResponse(call: Call, response: Response) {
                if (!response.isSuccessful) {
                    callback(null)
                    return
                }

                val responseBody = response.body?.string()
                try {
                    val reply = JSONObject(responseBody ?: "").optString("reply")
                    callback(reply)
                } catch (e: JSONException) {
                    Log.e("ClientManager", "Invalid JSON: $responseBody")
                    callback(null)
                }
            }

        })
    }

    /**
     * sends the phase in the format of JSON: {phase: [String]} to the server
     * to get the answer from the LLM
     * @param qiContext the qi Context
     * @param phase a [String] containing the input from the user
     * @param a callback that returns a String containing a [boolean] that says if the call is successful
     *
     * @return nothing
     */
    @JvmStatic
    fun postPhase(qiContext: QiContext, phase: String, callback: (Boolean) -> Unit) {
        val json = JSONObject().put("phase", phase)
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val body = RequestBody.create("application/json".toMediaTypeOrNull(), json.toString())
        val request = Request.Builder()
            .url("$baseUrl/phase")
            .post(body)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(false)
            override fun onResponse(call: Call, response: Response) {
                callback(response.isSuccessful)
            }
        })
    }

    /**
     * gets the phase in the format of JSON: {phase: [String]} from the server
     * @param qiContext the qi Context
     * @param a callback that returns a String containing a [String] that represents the phase
     *
     * @return nothing
     */
    @JvmStatic
    fun getPhase(qiContext: QiContext, callback: (String?) -> Unit) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val request = Request.Builder()
            .url("$baseUrl/phase")
            .get()
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(null)
            override fun onResponse(call: Call, response: Response) {
                val json = JSONObject(response.body?.string() ?: "")
                callback(json.optString("phase"))
            }
        })
    }

    /**
     * sends the language in the format of JSON: {phase: [String]} to the server
     * @param qiContext the qi Context
     * @param languageCode a [Int] containing the required language: 0: Italian, 1: English, 2: German
     * @param a callback that returns a String containing a [boolean] that says if the call is successful
     *
     * @return nothing
     */
    @JvmStatic
    fun postLanguage(qiContext: QiContext, langCode: Int, callback: (Boolean) -> Unit) {
        val json = JSONObject().put("language", langCode)
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val body = RequestBody.create("application/json".toMediaTypeOrNull(), json.toString())
        val request = Request.Builder()
            .url("$baseUrl/language")
            .post(body)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(false)
            override fun onResponse(call: Call, response: Response) {
                callback(response.isSuccessful)
            }
        })
    }

    /**
     * gets the language in the format of JSON: {language: [String]} from the server
     * @param qiContext the qi Context
     * @param a callback that returns a String containing a [String] that represents the language
     *
     * @return nothing
     */
    @JvmStatic
    fun getLanguage(qiContext: QiContext, callback: (String?) -> Unit) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val request = Request.Builder()
            .url("$baseUrl/language")
            .get()
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(null)
            override fun onResponse(call: Call, response: Response) {
                val json = JSONObject(response.body?.string() ?: "")
                callback(json.optString("language"))
            }
        })
    }

    /**
     * sends the language in the format of JSON: {language: [String]} to the server
     * @param qiContext the qi Context
     * @param text a [String] containing the required language style
     * @param a callback that returns a String containing a [boolean] that says if the call is successful
     *
     * @return nothing
     */
    @JvmStatic
    fun postLanguageStyle(qiContext: QiContext, text: String, callback: (Boolean) -> Unit) {
        val json = JSONObject().put("text", text)
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val body = RequestBody.create("application/json".toMediaTypeOrNull(), json.toString())
        val request = Request.Builder()
            .url("$baseUrl/language_style")
            .post(body)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(false)
            override fun onResponse(call: Call, response: Response) {
                callback(response.isSuccessful)
            }
        })
    }

    /**
     * gets the language style in the format of JSON: {language_style: [String]} from the server
     * @param qiContext the qi Context
     * @param a callback that returns a String containing a [String] that represents the language style
     *
     * @return nothing
     */
    fun getLanguageStyle(qiContext: QiContext, callback: (String?) -> Unit) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val request = Request.Builder()
            .url("$baseUrl/language_style")
            .get()
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(null)
            override fun onResponse(call: Call, response: Response) {
                val json = JSONObject(response.body?.string() ?: "")
                callback(json.optString("language_style"))
            }
        })
    }

    /**
     * sends the language in the format of JSON: {text: [String]} to the server
     * @param qiContext the qi Context
     * @param text a [String] containing the change in the proxemics
     * @param a callback that returns a String containing a [boolean] that says if the call is successful
     *
     * @return nothing
     */
    fun postProxemics(qiContext: QiContext, prox: Float, callback: (Boolean) -> Unit) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val json = JSONObject().put("proxemics", prox)
        val body = RequestBody.create("application/json".toMediaTypeOrNull(), json.toString())
        val request = Request.Builder()
            .url("$baseUrl/proxemics")
            .post(body)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(false)
            override fun onResponse(call: Call, response: Response) {
                callback(response.isSuccessful)
            }
        })
    }

    /**
     * gets the proxemics in the format of JSON: {language_style: [String]} from the server
     * @param qiContext the qi Context
     * @param a callback that returns a String containing a [Float] that represents the proxemics value
     *
     * @return nothing
     */
    fun getProxemics(qiContext: QiContext, callback: (Float?) -> Unit) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val request = Request.Builder()
            .url("$baseUrl/proxemics")
            .get()
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(null)
            override fun onResponse(call: Call, response: Response) {
                val json = JSONObject(response.body?.string() ?: "")
                callback(json.optDouble("proxemics").toFloat())
            }
        })
    }

    /**
     * gets the status of the Conversation Manager in the format of JSON:
     * {model: [String], language: [String], phase: [String], proxemics: [Float], style: [String]}
     * @param qiContext the qi Context
     * @param a callback that returns a String containing a [String] that represents the previous JSON
     *
     * @return nothing
     */
    fun getStatus(qiContext: QiContext, callback: (JSONObject?) -> Unit) {
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val request = Request.Builder()
            .url("$baseUrl/status")
            .get()
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(null)
            override fun onResponse(call: Call, response: Response) {
                val body = response.body?.string()
                val json = JSONObject(body ?: "{}")
                callback(json)
            }
        })
    }

    /**
     * sends the language in the format of JSON: {paradigm: [String], culture: [String]} to the server
     * @param qiContext the qi Context
     * @param culture a [String] containing the required language: Italian, English, German
     * @param paradigm a [String] containing the required paradigm: baseline, foreknowledge, adaptation
     * @param a callback that returns a String containing a [boolean] that says if the call is successful
     *
     * @return nothing
     */
    @JvmStatic
    fun postParadigm(qiContext: QiContext, culture: String, paradigm: String, callback: (Boolean) -> Unit) {
        val json = JSONObject().apply{
            put("culture", culture)
            put("paradigm", paradigm)
        }

        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val body = RequestBody.create("application/json".toMediaTypeOrNull(), json.toString())
        val request = Request.Builder()
            .url("$baseUrl/paradigm")
            .post(body)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(false)
            override fun onResponse(call: Call, response: Response) {
                callback(response.isSuccessful)
            }
        })
    }

    /**
     * resets the Conversation Manager
     * @param qiContext the qi Context
     * @param a callback that returns a String containing a [boolean] that says if the call is successful
     *
     * @return nothing
     */
    @JvmStatic
    fun reset(qiContext: QiContext, callback: (Boolean) -> Unit) {


        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val emptyBody = RequestBody.create(null, ByteArray(0))
        val request = Request.Builder()
            .url("$baseUrl/reset")
            .post(emptyBody)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(false)
            override fun onResponse(call: Call, response: Response) {
                callback(response.isSuccessful)
            }
        })
    }

    @JvmStatic
    fun checkProxemicChange(context: Context, callback: (Boolean, JSONArray?, JSONArray?, JSONArray?) -> Unit) {
        val prefs = context.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", null) ?: return callback(false, null, null, null)

        val request = Request.Builder()
            .url("$baseUrl/is_proxemic_changed")
            .get()
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                callback(false, null, null, null)
            }

            override fun onResponse(call: Call, response: Response) {
                if (!response.isSuccessful) {
                    callback(false, null, null, null)
                    return
                }

                val json = JSONObject(response.body?.string() ?: "")
                val changed = json.optBoolean("changed", false)
                val nodes = json.optJSONArray("nodes")
                val optimalPath = json.optJSONArray("optimal_path")
                val smoothPath = json.optJSONArray("smooth_path")
                callback(changed, nodes, optimalPath, smoothPath)
            }
        })
    }


    @JvmStatic
    fun getProxemicChanged(qiContext: QiContext): JSONObject {
        val prefs: SharedPreferences = qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val request = Request.Builder()
            .url("$baseUrl/is_proxemic_changed")
            .get()
            .build()

        val latch = CountDownLatch(1)
        var json_obj = JSONObject()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) {
                Log.e("ClientManager", "Network error: ${e.message}")
                latch.countDown()
            }

            override fun onResponse(call: Call, response: Response) {
                try {
                    if (response.isSuccessful) {
                        val body = response.body?.string()
                        json_obj = JSONObject(body ?: "{}")
                        Log.d("ClientManager", "Received JSON: $json_obj")
                    } else {
                        Log.e("ClientManager", "Request failed with code ${response.code}")
                    }
                } catch (e: Exception) {
                    Log.e("ClientManager", "Exception parsing response: ${e.message}")
                } finally {
                    latch.countDown()
                }
            }
        })

        latch.await() // Wait for response
        return json_obj
    }

    @JvmStatic
    fun postSetCulture(qiContext: QiContext, culture: String, callback: (Boolean) -> Unit) {
        val json = JSONObject().put("culture", culture)
        val prefs: SharedPreferences =
            qiContext.getSharedPreferences("settings", Context.MODE_PRIVATE)
        val baseUrl = prefs.getString("server_url", qiContext.getString(R.string.serverUrl))
        val body = RequestBody.create("application/json".toMediaTypeOrNull(), json.toString())
        val request = Request.Builder()
            .url("$baseUrl/set_culture")
            .post(body)
            .build()

        client.newCall(request).enqueue(object : Callback {
            override fun onFailure(call: Call, e: IOException) = callback(false)
            override fun onResponse(call: Call, response: Response) {
                if (!response.isSuccessful) {
                    callback(false)
                    return
                }

                val responseBody = response.body?.string()
                try {
                    val status = JSONObject(responseBody ?: "").optString("status")
                    callback(status == "success")
                } catch (e: JSONException) {
                    Log.e("ClientManager", "Invalid JSON: $responseBody")
                    callback(false)
                }
            }
        })
    }
}