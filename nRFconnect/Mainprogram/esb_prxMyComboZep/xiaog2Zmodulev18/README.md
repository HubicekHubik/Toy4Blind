# Edge Impulse model for Zephyr
Model package name: XiaoG2
Model version: 18
Edge Impulse SDK version: v1.79.0

## Instructions
To add the model:
Copy model-parameters, tflite-model and zephyr folders and CMakeList.txt inside your project.
Then, add the following line to your CMakeList.txt (need to be done just once!):
list(APPEND ZEPHYR_EXTRA_MODULES /model_folder)
where model_folder is the path to where you copied the folders extracted.

To add the Edge Impulse SDK to your project, copy the the entry of edge-impulse-sdk-zephyr from the extracted `west.yml` into your manifest (of zephyr or your project, depends on the topology in use) starting from the line to reference the Edge Impulse SDK version needed for this model.
You need to copy:
```
    - name: edge-impulse-sdk-zephyr
      description: Edge Impulse SDK for Zephyr
      path: modules/edge-impulse-sdk-zephyr
      revision: v1.79.0
      url: https://github.com/edgeimpulse/edge-impulse-sdk-zephyr
```

Check if the revision of the `edge-impulse-sdk-zephyr` module in your manifest file matches with the one needed for this model which is v1.79.0.
If is not the case, or is it the first time you integrate the Edge Impulse SDK, update your manifest file with the above entry and then call `west update` to pull the required version.

Happy inferencing!

