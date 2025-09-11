Time Series Analytics
=====================

The Time Series Analytics microservice provides the analytics capabilities for a time series use case.

It is a powerful, flexible solution for real-time analysis of time series data. Built on top of **Kapacitor**, it enables both streaming and batch processing, seamlessly integrating with **InfluxDB** for efficient data storage and retrieval.

What sets this microservice apart is its support for advanced analytics through **User-Defined Functions (UDFs)** written in Python. By leveraging the Intel® Extension for Scikit-learn*, you can accelerate machine learning workloads within their UDFs, unlocking high-performance anomaly detection, predictive maintenance, and other sophisticated analytics.

The key features include:
- **Bring your own Data Sets and corresponding User Defined Functions(UDFs) for custom analytics**: Easily implement and deploy your own Python-based analytics logic, following Kapacitor’s UDF standards.
- **Seamless Integration**: Automatically stores processed results back into InfluxDB for unified data management and visualization.
- **Model Registry Support**: Dynamically fetch and deploy UDF scripts, machine learning models, and TICKscripts from the Model Registry microservice, enabling rapid customization and iteration.
- **Versatile Use Cases**: Ideal for anomaly detection, alerting, and advanced time series analytics in industrial, IoT, and enterprise environments.

For more information on creating custom UDFs, see the
`Kapacitor Anomaly Detection Guide <https://docs.influxdata.com/kapacitor/v1/guides/anomaly_detection/>`__


.. toctree::
   :hidden:

   high-level-arch
   system-requirements
   get-started
   how-to-access-api
   how-to-deploy-with-helm
   api-reference
   release_notes/Overview