a = {'10': ['1'], '30': [], '60': [], '120': [], '180': [1,2,34,5], '300': []}


for key, value in a.items():
    if key == '180':
        print(a[key])


for lst in a.values():
    if lst:
        print('inside')
    else:
        print(f'{lst} empty')