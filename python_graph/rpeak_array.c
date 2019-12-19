#include "main.h"
const int RPEAK_LENGTH = 608;
const int RpeakSamples[RPEAK_LENGTH] = {
//18 - describes beat change: (N 
77,
370,
663, //1
947, //1
1231,
1515,
1809,
2045, //1
2403, //1
2706,
2998,
3283, //1
3560,
3863, //1
4171, //1
4466,
4765, //1
5061, //1
5347, //1
5634, //1
5918,
6215, //1
6527,
6824, //1
7106,
7393, //2
7670,
7953,
8246, //1
8539,
8837,
9142, //1
9432, //1
9710,
9998,
10283, //1
10591,
10895, //1
11191,
11480,
11781,
12066,
12351, //1
12645,
12950, //1
13267, //1
13562,
13842,
14131,
14423,
14711, //1
15012, //1
15310,
15607,
15900, //1
16183,
16465, //1
16755,
17058,
17359, //1
17657,
17947,
18227,
18514,
18796, //1
19081, //1
19389, //1
19694, //1
19989,
20272, //1
20554,
20838, //1
21131,
21424, //1
21729,
22030, //1
22321,
22603,
22881,
23164,
23454, //1
23757, //1
24053,
24346, //1
24626, //1
24914, //1
25198, //1
25485,
25780, //1
26089, //1
26387,
26671, //1
26952,
27238, //1
27536,
27833,
28133, //1
28431,
28727,
29015, //1
29294,
29581, //1
29873,
30182,
30487,
30779,
31065,
31349, //1
31636, //1
31928, //1
32225, //1
32531, //2
32836,
33127,
33404, //1
33692, //1
33981, //1
34274,
34575,
34870,
35168,
35455,
35736,
36016,
36310, //1
36605,
36916,
37215,
37500, //1
37783, //1
38071,
38356,
38652, //1
38950,
39252,
39548, //1
39826, //1
40096,
40382,
40678, //1
40970,
41272, //1
41567,
41850, //1
42117,
42416,
42698, //1
42997, //1
43307,
43604, //1
43893, //1
44172,
44456, //1
44743,
45030,
45323,
45628,
45925,
46204,
46479, //1
46760, //1
47037,
47335, //1
47633, //1
47920, //1
48202,
48486,
48766,
49040,
49323,
49618,
49923,
50214,
50491,
50771,
51056,
51340, //1
51627, //1
51921,
52216,
52506,
52784,
53063,
53341,
53632, //1
53923,
54220, //1
54508, //1
54784,
55064,
55344, //1
55626, //1
55908,
56208, //1
56502,
56785, //1
57055,
57333,
57616, //1
57901, //1
58192,
58490,
58788,
59077,
59354,
59632, //1
59920, //1
60214,
60516, //1
60818, //1
61106, //1
61391,
61680,
61964, //1
62248, //1
62547,
62852,
63155, //1
63439, //1
63711,
63999, //1
64288,
64582, //1
64876,
65174,
65464,
65750,
66031, //1
66308, //1
66604,
66792,
67131, //1
67434,
67728,
68008, //1
68302,
68595, //1
68886,
69191,
69504, //1
69796,
70074, //1
70357,
70643,
70941,
71240,
71540, //1
71846, //1
72138,
72417,
72703,
72998,
73301,
73612, //1
73912, //1
74196,
74483,
74767,
74986,
75332,
75632,
75936, //2
76234, //1
76515,
76786, //1
77074,
77367, //1
77664, //1
77954,
78251,
78539,
78825, //1
79100,
79386,
79689,
79992, //1
80291, //1
80583,
80869, //1
81152,
81439,
81731, //1
82031, //1
82336, //1
82637, //1
82928, //1
83209,
83492,
83795, //1
84104, //1
84407,
84710, //1
85010,
85303,
85580,
85870,
86172, //1
86478,
86780,
87079,
87364,
87652,
87941,
88232,
88530,
88834,
89136, //1
89421,
89704, //1
89988,
90286,
90590,
90885,
91190,
91482, //1
91761,
92047, //1
92338,
92633,
92945, //1
93246,
93534,
93824,
94103, //1
94396, //1
94693,
94996,
95299, //1
95607, //1
95893,
96171,
96451, //1
96747, //1
97050,
97348,
97645, //1
97937, //1
98228,
98508,
98789,
99084,
99382,
99580, //1
99930,
100218,
100496,
100781,
101071,
101361,
101654, //1
101942, //1
102235, //1
102517, //1
102793,
103074,
103371,
103669,
103964,
104262, //1
104546, //1
104831, //1
105118,
105407,
105707,
106016, //1
106315, //1
106600,
106882,
107159,
107455, //2
107752, //2
108045,
108342,
108644, //1
108926,
109199,
109486,
109772,
110077, //1
110375, //1
110674, //1
110963,
111244,
111524,
111811, //1
112108,
112405,
112711, //1
113001, //1
113281,
113563, //1
113850,
114142,
114428,
114716,
115001, //1
115280,
115547,
115815,
116084,
116370, //1
116654,
116940, //1
117214, //1
117494,
117766,
118044,
118323,
118607,
118912, //1
119216, //1
119510, //1
119792, //1
120077,
120376,
120676,
120991, //1
121301, //1
121605, //1
121890, //1
122170,
122461, //1
122751, //1
123050,
123360,
123664,
123936,
124222, //1
124507,
124776, //1
125060,
125355,
125649, //1
125926,
126204, //1
126476,
126747, //1
127025,
127314, //1
127604, //1
127894, //2
128086, //1
128423,
128698,
128958,
129231, //1
129520, //1
129799, //1
130057,
130318,
130568,
130819, //1
131074, //1
131327, //1
131585, //1
131853, //2
132122,
132381,
132629,
132883,
133132,
133390,
133663, //1
133933,
134197,
134464, //1
134738, //1
134990,
135243,
135505, //1
135781, //1
136061,
136351, //1
136630,
136899,
137171, //1
137449,
137729,
138008,
138304,
138599, //1
138889, //1
139164,
139440,
139720, //1
140010, //1
140302,
140599,
140882, //1
141152,
141423,
141684,
141943,
142207,
142480, //1
142747,
143011, //1
143265,
143515,
143766,
144025,
144286,
144558, //1
144832,
145112, //1
145382, //1
145647,
145903, //1
146170,
146446, //1
146737,
147013,
147290,
147565,
147833, //1
148099,
148365, //2
148638, //1
148916,
149205, //1
149488,
149769, //1
150030,
150304, //1
150576, //1
150849,
151123, //1
151407,
151700,
151974,
152237,
152511, //1
152782, //1
153067,
153362,
153644,
153923, //1
154193,
154471, //1
154742, //1
155009,
155292, //1
155589, //1
155865,
156133, //1
156395,
156648,
156906,
157169, //1
157431, //1
157696, //1
157960,
158234, //1
158490, //1
158730,
158974,
159226, //1
159480, //1
159738,
160002, //1
160260,
160507,
160755,
161005,
161255, //1
161501, //2
161765, //1
162036, //1
162309, //1
162574, //1
162836, //1
163093,
163363, //1
163630, //1
163899,
164182,
164461,
164731,
164992, //1
165246,
165499,
165756,
166033, //1
166301,
166570,
166842, //1
167112, //1
167371,
167641,
167912,
168191,
168485, //1
168785, //1
169074,
169349,
169630, //1
169907,
170202, //1
170496, //1
170719,
171074,
171373, //2
171653, //1
171921,
172200, //1
172482, //1
172776,
};
