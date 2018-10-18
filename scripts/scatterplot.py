import numpy as np
import plotly.plotly as py
import plotly.graph_objs as go
import plotly.tools as tls
import plotly.figure_factory as ff
import yaml

import copy
import pandas as pd

tls.set_credentials_file(username='magnusgaertner', api_key='15MA0eA0T4eGNANsrfPo')
fp = open("ergebnisse1711_2.yaml", "r")
results_unfiltered = yaml.load(fp)

results = [result for result in results_unfiltered if  int(result['first_valid_iteration'])!=-1]
print(len(results))
print(results[0])
# for dev_index in [0,1,2,3]:
#   print([len([1 for r in results if r['std_devs'][dev_index] == dev]) for dev in [0.05, 0.1, 0.15, 0.2, 0.4]])
#
# # dev1 0.2 / 0.4 , dev2 0.15, dev 3 0.15, dev 4 = 0.4
# results = [result for result in results_unfiltered if result['std_devs'][1:4] == [0.15, 0.15, 0.4] and float(result['std_devs'][0]) > 0.18]
# print(len(results))
# print(results[0])
classes = range(1, 21)
pl_colorscale = [[0.0, '#19d3f3'],
                 [0.333, '#19d3f3'],
                 [0.333, '#e763fa'],
                 [0.666, '#e763fa'],
                 [0.666, '#636efa'],
                 [1, '#636efa']]

text = range(0, len(results))
num_iterations = [int(result['first_valid_iteration']) % 22 for result in results]
time = [float(result['time']) for result in results]
dimensions = [
  dict(label="dev1",
       values=[result['std_devs'][0] for result in results]),
  dict(label="dev2",
       values=[result['std_devs'][1] for result in results]),
  dict(label="dev3",
       values=[result['std_devs'][2] for result in results]),
  dict(label="dev4",
       values=[result['std_devs'][3] for result in results]),
  dict(label="dev5",
       values=[result['std_devs'][4] for result in results]),
  dict(label="dev6",
       values=[result['std_devs'][5] for result in results]),
  dict(label="num_rollouts",
       values=[result['params']['num_rollouts'] for result in results]),
  #dict(label="max_rollouts",
  #     values=[result['params']['max_rollouts'] for result in results]),
  dict(label="num_timesteps",
       values=[result['params']['num_timesteps'] for result in results]),
  #dict(label="num_iterations_after_valid",
  #     values=[result['params']['num_iterations_after_valid'] for result in results]),
  dict(label="num_iterations",
       values=num_iterations),
  dict(label="cost",
       values=[0 if len(result['cost']) == 0 else result['cost'][-1] for result in results]),
  dict(label="time",
       values=time)]

trace1 = go.Splom(dimensions=dimensions,
                  text=text,
                  marker=dict(#color=time,
                              size=7, # [(30-iteration)/3 for iteration in num_iterations],
                              # colorscale=pl_colorscale,
                              showscale=False,
                              line=dict(width=0.5,
                                        color='rgb(230,230,230)')),
                  showupperhalf=False)

axis = dict(showline=True,
            zeroline=False,
            gridcolor="#fff",
            ticklen=4)
layout = go.Layout(
  title='stomp std dev evaluation',
  dragmode='select',
  width=1500,
  height=1500,
  autosize=True,
  hovermode='closest',
  plot_bgcolor='rgba(240,240,240, 0.95)',
  xaxis1=dict(axis),
  xaxis2=dict(axis),
  xaxis3=dict(axis),
  xaxis4=dict(axis),
  yaxis1=dict(axis),
  yaxis2=dict(axis),
  yaxis3=dict(axis),
  yaxis4=dict(axis))


fig1 = dict(data=[trace1], layout=layout)
py.iplot(fig1, filename='std_devs_stomp')
