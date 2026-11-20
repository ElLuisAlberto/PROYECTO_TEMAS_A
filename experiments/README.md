# Experimentos

Aquí se guardarán perfiles de red, hojas de ejecución y resultados
reproducibles. No se deben mezclar capturas o archivos temporales sin contexto.

Para cada ensayo se creará una hoja en run_sheets con:

~~~
run_id, fecha, escenario, participantes, reparto de ejes,
perfil de red, parámetros, versión del código, resultado,
rutas a logs/video y observaciones
~~~

Los logs JSONL pueden resumirse sin dependencias externas con:

~~~bash
PYTHONPATH=src python3 scripts/summarize_run_log.py \
  evidence/logs/RUN_ID.jsonl --output experiments/results/RUN_ID-summary.json
~~~

Los perfiles de red se documentarán en network_profiles y los resultados
procesados se guardarán en results.
