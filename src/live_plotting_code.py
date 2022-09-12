def plotter(self):
        print("TEST")
        # define and adjust figure
        fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
        self.ax_vx = plt.subplot(241)
        self.ax_vy = plt.subplot(242)
        self.ax_vz = plt.subplot(243)
        self.ax_yaw_rate = plt.subplot(244)
        self.ax_vx.set_facecolor('#DEDEDE')
        self.ax_vy.set_facecolor('#DEDEDE')
        self.ax_vz.set_facecolor('#DEDEDE')
        self.ax_yaw_rate.set_facecolor('#DEDEDE')

        # animate
        ani = FuncAnimation(fig, self.update_plot, interval=1000)

        plt.show(block=False)
        plt.draw()

    def update_plot(self, i):

        print("TEST")
        # update the data
        self.vx_store.popleft()
        self.vy_store.popleft()
        self.vz_store.popleft()
        self.yaw_rate_store.popleft()
        self.vx_store.append(self.vx)
        self.vy_store.append(self.vy)
        self.vz_store.append(self.vz)
        self.yaw_rate_store.append(self.yaw_rate)

        # clear axis
        self.ax_vx.cla()
        self.ax_vy.cla()
        self.ax_vz.cla()
        self.ax_yaw_rate.cla()

        # plot
        self.ax_vx.plot(self.vx_store)
        self.ax_vx.scatter(len(self.vx_store)-1, self.vx_store[-1])
        self.ax_vx.text(len(self.vx_store)-1, self.vx_store[-1]+2, "{}%".format(self.vx_store[-1]))
        self.ax_vx.set_ylim(0,100)
        self.ax_vy.plot(self.vy_store)
        self.ax_vy.scatter(len(self.vy_store)-1, self.vy_store[-1])
        self.ax_vy.text(len(self.vy_store)-1, self.vy_store[-1]+2, "{}%".format(self.vy_store[-1]))
        self.ax_vy.set_ylim(0,100)
        self.ax_vz.plot(self.vz_store)
        self.ax_vz.scatter(len(self.vz_store)-1, self.vz_store[-1])
        self.ax_vz.text(len(self.vz_store)-1, self.vz_store[-1]+2, "{}%".format(self.vz_store[-1]))
        self.ax_vz.set_ylim(0,100)
        self.ax_yaw_rate.plot(self.yaw_rate_store)
        self.ax_yaw_rate.scatter(len(self.yaw_rate_store)-1, self.yaw_rate_store[-1])
        self.ax_yaw_rate.text(len(self.yaw_rate_store)-1, self.yaw_rate_store[-1]+2, "{}%".format(self.yaw_rate_store[-1]))
        self.ax_yaw_rate.set_ylim(0,100)